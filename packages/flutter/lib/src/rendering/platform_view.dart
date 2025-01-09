// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';

import 'box.dart';
import 'layer.dart';
import 'object.dart';

/// How an embedded platform view behave during hit tests.
enum PlatformViewHitTestBehavior {
  /// Opaque targets can be hit by hit tests, causing them to both receive
  /// events within their bounds and prevent targets visually behind them from
  /// also receiving events.
  opaque,

  /// Translucent targets both receive events within their bounds and permit
  /// targets visually behind them to also receive events.
  translucent,

  /// Transparent targets don't receive events within their bounds and permit
  /// targets visually behind them to receive events.
  transparent,
}

enum _PlatformViewState { uninitialized, resizing, ready }

bool _factoryTypesSetEquals<T>(Set<Factory<T>>? a, Set<Factory<T>>? b) {
  if (a == b) {
    return true;
  }
  if (a == null || b == null) {
    return false;
  }
  return setEquals(_factoriesTypeSet(a), _factoriesTypeSet(b));
}

Set<Type> _factoriesTypeSet<T>(Set<Factory<T>> factories) {
  return factories.map<Type>((Factory<T> factory) => factory.type).toSet();
}

/// A render object for an Android view.
///
/// Requires Android API level 23 or greater.
///
/// [RenderAndroidView] is responsible for sizing, displaying and passing touch events to an
/// Android [View](https://developer.android.com/reference/android/view/View).
///
/// {@template flutter.rendering.RenderAndroidView.layout}
/// The render object's layout behavior is to fill all available space, the parent of this object must
/// provide bounded layout constraints.
/// {@endtemplate}
///
/// {@template flutter.rendering.RenderAndroidView.gestures}
/// The render object participates in Flutter's gesture arenas, and dispatches touch events to the
/// platform view iff it won the arena. Specific gestures that should be dispatched to the platform
/// view can be specified with factories in the `gestureRecognizers` constructor parameter or
/// by calling `updateGestureRecognizers`. If the set of gesture recognizers is empty, the gesture
/// will be dispatched to the platform view iff it was not claimed by any other gesture recognizer.
/// {@endtemplate}
///
/// See also:
///
///  * [AndroidView] which is a widget that is used to show an Android view.
///  * [PlatformViewsService] which is a service for controlling platform views.
class RenderAndroidView extends PlatformViewRenderBox {
  /// Creates a render object for an Android view.
  RenderAndroidView({
    required AndroidViewController viewController,
    required PlatformViewHitTestBehavior hitTestBehavior,
    required Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers,
    Clip clipBehavior = Clip.hardEdge,
  }) : _viewController = viewController,
       _clipBehavior = clipBehavior,
       super(
         controller: viewController,
         hitTestBehavior: hitTestBehavior,
         gestureRecognizers: gestureRecognizers,
       ) {
    _viewController.pointTransformer = (Offset offset) => globalToLocal(offset);
    updateGestureRecognizers(gestureRecognizers);
    _viewController.addOnPlatformViewCreatedListener(_onPlatformViewCreated);
    this.hitTestBehavior = hitTestBehavior;
    _setOffset();
  }

  _PlatformViewState _state = _PlatformViewState.uninitialized;

  Size? _currentTextureSize;

  bool _isDisposed = false;

  /// The Android view controller for the Android view associated with this render object.
  @override
  AndroidViewController get controller => _viewController;

  AndroidViewController _viewController;

  /// Sets a new Android view controller.
  @override
  set controller(AndroidViewController controller) {
    assert(!_isDisposed);
    if (_viewController == controller) {
      return;
    }
    _viewController.removeOnPlatformViewCreatedListener(_onPlatformViewCreated);
    super.controller = controller;
    _viewController = controller;
    _viewController.pointTransformer = (Offset offset) => globalToLocal(offset);
    _sizePlatformView();
    if (_viewController.isCreated) {
      markNeedsSemanticsUpdate();
    }
    _viewController.addOnPlatformViewCreatedListener(_onPlatformViewCreated);
  }

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior = Clip.hardEdge;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  void _onPlatformViewCreated(int id) {
    assert(!_isDisposed);
    markNeedsSemanticsUpdate();
  }

  @override
  bool get sizedByParent => true;

  @override
  bool get alwaysNeedsCompositing => true;

  @override
  bool get isRepaintBoundary => true;

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return constraints.biggest;
  }

  @override
  void performResize() {
    super.performResize();
    _sizePlatformView();
  }

  Future<void> _sizePlatformView() async {
    // Android virtual displays cannot have a zero size.
    // Trying to size it to 0 crashes the app, which was happening when starting the app
    // with a locked screen (see: https://github.com/flutter/flutter/issues/20456).
    if (_state == _PlatformViewState.resizing || size.isEmpty) {
      return;
    }

    _state = _PlatformViewState.resizing;
    markNeedsPaint();

    Size targetSize;
    do {
      targetSize = size;
      _currentTextureSize = await _viewController.setSize(targetSize);
      if (_isDisposed) {
        return;
      }
      // We've resized the platform view to targetSize, but it is possible that
      // while we were resizing the render object's size was changed again.
      // In that case we will resize the platform view again.
    } while (size != targetSize);

    _state = _PlatformViewState.ready;
    markNeedsPaint();
  }

  // Sets the offset of the underlying platform view on the platform side.
  //
  // This allows the Android native view to draw the a11y highlights in the same
  // location on the screen as the platform view widget in the Flutter framework.
  //
  // It also allows platform code to obtain the correct position of the Android
  // native view on the screen.
  void _setOffset() {
    SchedulerBinding.instance.addPostFrameCallback((_) async {
      if (!_isDisposed) {
        if (attached) {
          await _viewController.setOffset(localToGlobal(Offset.zero));
        }
        // Schedule a new post frame callback.
        _setOffset();
      }
    }, debugLabel: 'RenderAndroidView.setOffset');
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (_viewController.textureId == null || _currentTextureSize == null) {
      return;
    }

    // As resizing the Android view happens asynchronously we don't know exactly when is a
    // texture frame with the new size is ready for consumption.
    // TextureLayer is unaware of the texture frame's size and always maps it to the
    // specified rect. If the rect we provide has a different size from the current texture frame's
    // size the texture frame will be scaled.
    // To prevent unwanted scaling artifacts while resizing, clip the texture.
    // This guarantees that the size of the texture frame we're painting is always
    // _currentAndroidTextureSize.
    final bool isTextureLargerThanWidget =
        _currentTextureSize!.width > size.width || _currentTextureSize!.height > size.height;
    if (isTextureLargerThanWidget && clipBehavior != Clip.none) {
      _clipRectLayer.layer = context.pushClipRect(
        true,
        offset,
        offset & size,
        _paintTexture,
        clipBehavior: clipBehavior,
        oldLayer: _clipRectLayer.layer,
      );
      return;
    }
    _clipRectLayer.layer = null;
    _paintTexture(context, offset);
  }

  final LayerHandle<ClipRectLayer> _clipRectLayer = LayerHandle<ClipRectLayer>();

  @override
  void dispose() {
    _isDisposed = true;
    _clipRectLayer.layer = null;
    _viewController.removeOnPlatformViewCreatedListener(_onPlatformViewCreated);
    super.dispose();
  }

  void _paintTexture(PaintingContext context, Offset offset) {
    if (_currentTextureSize == null) {
      return;
    }

    context.addLayer(
      TextureLayer(rect: offset & _currentTextureSize!, textureId: _viewController.textureId!),
    );
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    // Don't call the super implementation since `platformViewId` should
    // be set only when the platform view is created, but the concept of
    // a "created" platform view belongs to this subclass.
    config.isSemanticBoundary = true;

    if (_viewController.isCreated) {
      config.platformViewId = _viewController.viewId;
    }
  }
}

/// Common render-layer functionality for iOS and macOS platform views.
///
/// Provides the basic rendering logic for iOS and macOS platformviews.
/// Subclasses shall override handleEvent in order to execute custom event logic.
/// T represents the class of the view controller for the corresponding widget.
abstract class RenderDarwinPlatformView<T extends DarwinPlatformViewController> extends RenderBox {
  /// Creates a render object for a platform view.
  RenderDarwinPlatformView({
    required T viewController,
    required this.hitTestBehavior,
    required Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers,
  }) : _viewController = viewController {
    updateGestureRecognizers(gestureRecognizers);
  }

  /// The unique identifier of the platform view controlled by this controller.
  T get viewController => _viewController;
  T _viewController;
  set viewController(T value) {
    if (_viewController == value) {
      return;
    }
    final bool needsSemanticsUpdate = _viewController.id != value.id;
    _viewController = value;
    markNeedsPaint();
    if (needsSemanticsUpdate) {
      markNeedsSemanticsUpdate();
    }
  }

  /// How to behave during hit testing.
  // The implicit setter is enough here as changing this value will just affect
  // any newly arriving events there's nothing we need to invalidate.
  PlatformViewHitTestBehavior hitTestBehavior;

  @override
  bool get sizedByParent => true;

  @override
  bool get alwaysNeedsCompositing => true;

  @override
  bool get isRepaintBoundary => true;

  PointerEvent? _lastPointerDownEvent;

  _UiKitViewGestureRecognizer? _gestureRecognizer;

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return constraints.biggest;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    context.addLayer(PlatformViewLayer(rect: offset & size, viewId: _viewController.id));
  }

  @override
  bool hitTest(BoxHitTestResult result, {Offset? position}) {
    if (hitTestBehavior == PlatformViewHitTestBehavior.transparent || !size.contains(position!)) {
      return false;
    }
    result.add(BoxHitTestEntry(this, position));
    return hitTestBehavior == PlatformViewHitTestBehavior.opaque;
  }

  @override
  bool hitTestSelf(Offset position) => hitTestBehavior != PlatformViewHitTestBehavior.transparent;

  // This is registered as a global PointerRoute while the render object is attached.
  void _handleGlobalPointerEvent(PointerEvent event) {
    if (event is! PointerDownEvent) {
      return;
    }
    if (!(Offset.zero & size).contains(globalToLocal(event.position))) {
      return;
    }
    if ((event.original ?? event) != _lastPointerDownEvent) {
      // The pointer event is in the bounds of this render box, but we didn't get it in handleEvent.
      // This means that the pointer event was absorbed by a different render object.
      // Since on the platform side the FlutterTouchIntercepting view is seeing all events that are
      // within its bounds we need to tell it to reject the current touch sequence.
      _viewController.rejectGesture();
    }
    _lastPointerDownEvent = null;
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.isSemanticBoundary = true;
    config.platformViewId = _viewController.id;
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    GestureBinding.instance.pointerRouter.addGlobalRoute(_handleGlobalPointerEvent);
  }

  @override
  void detach() {
    GestureBinding.instance.pointerRouter.removeGlobalRoute(_handleGlobalPointerEvent);
    super.detach();
  }

  /// {@macro flutter.rendering.PlatformViewRenderBox.updateGestureRecognizers}
  void updateGestureRecognizers(Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers);
}

/// A render object for an iOS UIKit UIView.
///
/// [RenderUiKitView] is responsible for sizing and displaying an iOS
/// [UIView](https://developer.apple.com/documentation/uikit/uiview).
///
/// UIViews are added as subviews of the FlutterView and are composited by Quartz.
///
/// The viewController is typically generated by [PlatformViewsRegistry.getNextPlatformViewId], the UIView
/// must have been created by calling [PlatformViewsService.initUiKitView].
///
/// {@macro flutter.rendering.RenderAndroidView.layout}
///
/// {@macro flutter.rendering.RenderAndroidView.gestures}
///
/// See also:
///
///  * [UiKitView], which is a widget that is used to show a UIView.
///  * [PlatformViewsService], which is a service for controlling platform views.
class RenderUiKitView extends RenderDarwinPlatformView<UiKitViewController> {
  /// Creates a render object for an iOS UIView.
  RenderUiKitView({
    required super.viewController,
    required super.hitTestBehavior,
    required super.gestureRecognizers,
  });

  /// {@macro flutter.rendering.PlatformViewRenderBox.updateGestureRecognizers}
  @override
  void updateGestureRecognizers(Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers) {
    assert(
      _factoriesTypeSet(gestureRecognizers).length == gestureRecognizers.length,
      'There were multiple gesture recognizer factories for the same type, there must only be a single '
      'gesture recognizer factory for each gesture recognizer type.',
    );
    if (_factoryTypesSetEquals(
      gestureRecognizers,
      _gestureRecognizer?.gestureRecognizerFactories,
    )) {
      return;
    }
    _gestureRecognizer?.dispose();
    _gestureRecognizer = _UiKitViewGestureRecognizer(viewController, gestureRecognizers);
  }

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    if (event is! PointerDownEvent) {
      return;
    }
    _gestureRecognizer!.addPointer(event);
    _lastPointerDownEvent = event.original ?? event;
  }

  @override
  void detach() {
    _gestureRecognizer!.reset();
    super.detach();
  }

  @override
  void dispose() {
    _gestureRecognizer?.dispose();
    super.dispose();
  }
}

/// A render object for a macOS platform view.
class RenderAppKitView extends RenderDarwinPlatformView<AppKitViewController> {
  /// Creates a render object for a macOS AppKitView.
  RenderAppKitView({
    required super.viewController,
    required super.hitTestBehavior,
    required super.gestureRecognizers,
  });

  // TODO(schectman): Add gesture functionality to macOS platform view when implemented.
  // https://github.com/flutter/flutter/issues/128519
  // This method will need to behave the same as the same-named method for RenderUiKitView,
  // but use a _AppKitViewGestureRecognizer or equivalent, whose constructor shall accept an
  // AppKitViewController.
  @override
  void updateGestureRecognizers(Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers) {}
}

// This recognizer constructs gesture recognizers from a set of gesture recognizer factories
// it was give, adds all of them to a gesture arena team with the _UiKitViewGestureRecognizer
// as the team captain.
// When the team wins a gesture the recognizer notifies the engine that it should release
// the touch sequence to the embedded UIView.
class _UiKitViewGestureRecognizer extends OneSequenceGestureRecognizer {
  _UiKitViewGestureRecognizer(this.controller, this.gestureRecognizerFactories) {
    team = GestureArenaTeam()..captain = this;
    _gestureRecognizers =
        gestureRecognizerFactories.map((Factory<OneSequenceGestureRecognizer> recognizerFactory) {
          final OneSequenceGestureRecognizer gestureRecognizer = recognizerFactory.constructor();
          gestureRecognizer.team = team;
          // The below gesture recognizers requires at least one non-empty callback to
          // compete in the gesture arena.
          // https://github.com/flutter/flutter/issues/35394#issuecomment-562285087
          if (gestureRecognizer is LongPressGestureRecognizer) {
            gestureRecognizer.onLongPress ??= () {};
          } else if (gestureRecognizer is DragGestureRecognizer) {
            gestureRecognizer.onDown ??= (_) {};
          } else if (gestureRecognizer is TapGestureRecognizer) {
            gestureRecognizer.onTapDown ??= (_) {};
          }
          return gestureRecognizer;
        }).toSet();
  }

  // We use OneSequenceGestureRecognizers as they support gesture arena teams.
  // TODO(amirh): get a list of GestureRecognizers here.
  // https://github.com/flutter/flutter/issues/20953
  final Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizerFactories;
  late Set<OneSequenceGestureRecognizer> _gestureRecognizers;

  final UiKitViewController controller;

  @override
  void addAllowedPointer(PointerDownEvent event) {
    super.addAllowedPointer(event);
    for (final OneSequenceGestureRecognizer recognizer in _gestureRecognizers) {
      recognizer.addPointer(event);
    }
  }

  @override
  String get debugDescription => 'UIKit view';

  @override
  void didStopTrackingLastPointer(int pointer) {}

  @override
  void handleEvent(PointerEvent event) {
    stopTrackingIfPointerNoLongerDown(event);
  }

  @override
  void acceptGesture(int pointer) {
    controller.acceptGesture();
  }

  @override
  void rejectGesture(int pointer) {
    controller.rejectGesture();
  }

  void reset() {
    resolve(GestureDisposition.rejected);
  }
}

typedef _HandlePointerEvent = Future<void> Function(PointerEvent event);

// This recognizer constructs gesture recognizers from a set of gesture recognizer factories
// it was give, adds all of them to a gesture arena team with the _PlatformViewGestureRecognizer
// as the team captain.
// As long as the gesture arena is unresolved, the recognizer caches all pointer events.
// When the team wins, the recognizer sends all the cached pointer events to `_handlePointerEvent`, and
// sets itself to a "forwarding mode" where it will forward any new pointer event to `_handlePointerEvent`.
class _PlatformViewGestureRecognizer extends OneSequenceGestureRecognizer {
  _PlatformViewGestureRecognizer(
    _HandlePointerEvent handlePointerEvent,
    this.gestureRecognizerFactories,
  ) {
    team = GestureArenaTeam()..captain = this;
    _gestureRecognizers =
        gestureRecognizerFactories.map((Factory<OneSequenceGestureRecognizer> recognizerFactory) {
          final OneSequenceGestureRecognizer gestureRecognizer = recognizerFactory.constructor();
          gestureRecognizer.team = team;
          // The below gesture recognizers requires at least one non-empty callback to
          // compete in the gesture arena.
          // https://github.com/flutter/flutter/issues/35394#issuecomment-562285087
          if (gestureRecognizer is LongPressGestureRecognizer) {
            gestureRecognizer.onLongPress ??= () {};
          } else if (gestureRecognizer is DragGestureRecognizer) {
            gestureRecognizer.onDown ??= (_) {};
          } else if (gestureRecognizer is TapGestureRecognizer) {
            gestureRecognizer.onTapDown ??= (_) {};
          }
          return gestureRecognizer;
        }).toSet();
    _handlePointerEvent = handlePointerEvent;
  }

  late _HandlePointerEvent _handlePointerEvent;

  // Maps a pointer to a list of its cached pointer events.
  // Before the arena for a pointer is resolved all events are cached here, if we win the arena
  // the cached events are dispatched to `_handlePointerEvent`, if we lose the arena we clear the cache for
  // the pointer.
  final Map<int, List<PointerEvent>> cachedEvents = <int, List<PointerEvent>>{};

  // Pointer for which we have already won the arena, events for pointers in this set are
  // immediately dispatched to `_handlePointerEvent`.
  final Set<int> forwardedPointers = <int>{};

  // We use OneSequenceGestureRecognizers as they support gesture arena teams.
  // TODO(amirh): get a list of GestureRecognizers here.
  // https://github.com/flutter/flutter/issues/20953
  final Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizerFactories;
  late Set<OneSequenceGestureRecognizer> _gestureRecognizers;

  @override
  void addAllowedPointer(PointerDownEvent event) {
    super.addAllowedPointer(event);
    for (final OneSequenceGestureRecognizer recognizer in _gestureRecognizers) {
      recognizer.addPointer(event);
    }
  }

  @override
  String get debugDescription => 'Platform view';

  @override
  void didStopTrackingLastPointer(int pointer) {}

  @override
  void handleEvent(PointerEvent event) {
    if (!forwardedPointers.contains(event.pointer)) {
      _cacheEvent(event);
    } else {
      _handlePointerEvent(event);
    }
    stopTrackingIfPointerNoLongerDown(event);
  }

  @override
  void acceptGesture(int pointer) {
    _flushPointerCache(pointer);
    forwardedPointers.add(pointer);
  }

  @override
  void rejectGesture(int pointer) {
    stopTrackingPointer(pointer);
    cachedEvents.remove(pointer);
  }

  void _cacheEvent(PointerEvent event) {
    if (!cachedEvents.containsKey(event.pointer)) {
      cachedEvents[event.pointer] = <PointerEvent>[];
    }
    cachedEvents[event.pointer]!.add(event);
  }

  void _flushPointerCache(int pointer) {
    cachedEvents.remove(pointer)?.forEach(_handlePointerEvent);
  }

  @override
  void stopTrackingPointer(int pointer) {
    super.stopTrackingPointer(pointer);
    forwardedPointers.remove(pointer);
  }

  void reset() {
    forwardedPointers.forEach(super.stopTrackingPointer);
    forwardedPointers.clear();
    cachedEvents.keys.forEach(super.stopTrackingPointer);
    cachedEvents.clear();
    resolve(GestureDisposition.rejected);
  }
}

/// A render object for embedding a platform view.
///
/// [PlatformViewRenderBox] presents a platform view by adding a [PlatformViewLayer] layer,
/// integrates it with the gesture arenas system and adds relevant semantic nodes to the semantics tree.
class PlatformViewRenderBox extends RenderBox with _PlatformViewGestureMixin {
  /// Creating a render object for a [PlatformViewSurface].
  PlatformViewRenderBox({
    required PlatformViewController controller,
    required PlatformViewHitTestBehavior hitTestBehavior,
    required Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers,
  }) : assert(controller.viewId > -1),
       _controller = controller {
    this.hitTestBehavior = hitTestBehavior;
    updateGestureRecognizers(gestureRecognizers);
  }

  /// The controller for this render object.
  PlatformViewController get controller => _controller;
  PlatformViewController _controller;

  /// Setting this value to a new value will result in a repaint.
  set controller(covariant PlatformViewController controller) {
    assert(controller.viewId > -1);

    if (_controller == controller) {
      return;
    }
    final bool needsSemanticsUpdate = _controller.viewId != controller.viewId;
    _controller = controller;
    markNeedsPaint();
    if (needsSemanticsUpdate) {
      markNeedsSemanticsUpdate();
    }
  }

  /// {@template flutter.rendering.PlatformViewRenderBox.updateGestureRecognizers}
  /// Updates which gestures should be forwarded to the platform view.
  ///
  /// Gesture recognizers created by factories in this set participate in the gesture arena for each
  /// pointer that was put down on the render box. If any of the recognizers on this list wins the
  /// gesture arena, the entire pointer event sequence starting from the pointer down event
  /// will be dispatched to the Android view.
  ///
  /// The `gestureRecognizers` property must not contain more than one factory with the same [Factory.type].
  ///
  /// Setting a new set of gesture recognizer factories with the same [Factory.type]s as the current
  /// set has no effect, because the factories' constructors would have already been called with the previous set.
  /// {@endtemplate}
  ///
  /// Any active gesture arena the `PlatformView` participates in is rejected when the
  /// set of gesture recognizers is changed.
  void updateGestureRecognizers(Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers) {
    _updateGestureRecognizersWithCallBack(gestureRecognizers, _controller.dispatchPointerEvent);
  }

  @override
  bool get sizedByParent => true;

  @override
  bool get alwaysNeedsCompositing => true;

  @override
  bool get isRepaintBoundary => true;

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return constraints.biggest;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    context.addLayer(PlatformViewLayer(rect: offset & size, viewId: _controller.viewId));
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.isSemanticBoundary = true;
    config.platformViewId = _controller.viewId;
  }
}

/// The Mixin handling the pointer events and gestures of a platform view render box.
mixin _PlatformViewGestureMixin on RenderBox implements MouseTrackerAnnotation {
  /// How to behave during hit testing.
  // Changing _hitTestBehavior might affect which objects are considered hovered over.
  set hitTestBehavior(PlatformViewHitTestBehavior value) {
    if (value != _hitTestBehavior) {
      _hitTestBehavior = value;
      if (owner != null) {
        markNeedsPaint();
      }
    }
  }

  PlatformViewHitTestBehavior? _hitTestBehavior;

  _HandlePointerEvent? _handlePointerEvent;

  /// {@macro flutter.rendering.PlatformViewRenderBox.updateGestureRecognizers}
  ///
  /// Any active gesture arena the `PlatformView` participates in is rejected when the
  /// set of gesture recognizers is changed.
  void _updateGestureRecognizersWithCallBack(
    Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers,
    _HandlePointerEvent handlePointerEvent,
  ) {
    assert(
      _factoriesTypeSet(gestureRecognizers).length == gestureRecognizers.length,
      'There were multiple gesture recognizer factories for the same type, there must only be a single '
      'gesture recognizer factory for each gesture recognizer type.',
    );
    if (_factoryTypesSetEquals(
      gestureRecognizers,
      _gestureRecognizer?.gestureRecognizerFactories,
    )) {
      return;
    }
    _gestureRecognizer?.dispose();
    _gestureRecognizer = _PlatformViewGestureRecognizer(handlePointerEvent, gestureRecognizers);
    _handlePointerEvent = handlePointerEvent;
  }

  _PlatformViewGestureRecognizer? _gestureRecognizer;

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (_hitTestBehavior == PlatformViewHitTestBehavior.transparent || !size.contains(position)) {
      return false;
    }
    result.add(BoxHitTestEntry(this, position));
    return _hitTestBehavior == PlatformViewHitTestBehavior.opaque;
  }

  @override
  bool hitTestSelf(Offset position) => _hitTestBehavior != PlatformViewHitTestBehavior.transparent;

  @override
  PointerEnterEventListener? get onEnter => null;

  @override
  PointerExitEventListener? get onExit => null;

  @override
  MouseCursor get cursor => MouseCursor.uncontrolled;

  @override
  bool get validForMouseTracker => true;

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    if (event is PointerDownEvent) {
      _gestureRecognizer!.addPointer(event);
    }
    if (event is PointerHoverEvent) {
      _handlePointerEvent?.call(event);
    }
  }

  @override
  void detach() {
    _gestureRecognizer!.reset();
    super.detach();
  }

  @override
  void dispose() {
    _gestureRecognizer?.dispose();
    super.dispose();
  }
}
