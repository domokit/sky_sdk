// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show FlutterView;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Defines the type of the Window.
enum WindowArchetype {
  /// Regular top-level window.
  regular,

  /// A window that is on a layer above regular windows and is not dockable.
  floating_regular,

  /// Dialog window.
  dialog,

  /// Satellite window attached to a regular, floating_regular or dialog window.
  satellite,

  /// Popup.
  popup,

  /// Tooltip.
  tip
}

/// Defines the anchor point for the anchor rectangle or child [Window] when
/// positioning a [Window]. The specified anchor is used to derive an anchor
/// point on the anchor rectangle that the anchor point for the child [Window]
/// will be positioned relative to. If a corner anchor is set (e.g. [topLeft]
/// or [bottomRight]), the anchor point will be at the specified corner;
/// otherwise, the derived anchor point will be centered on the specified edge,
/// or in the center of the anchor rectangle if no edge is specified.
enum WindowPositionerAnchor {
  /// If the [WindowPositioner.parentAnchor] is set to [center], then the
  /// child [Window] will be positioned relative to the center
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [center], then the middle
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  center,

  /// If the [WindowPositioner.parentAnchor] is set to [top], then the
  /// child [Window] will be positioned relative to the top
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [top], then the top
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  top,

  /// If the [WindowPositioner.parentAnchor] is set to [bottom], then the
  /// child [Window] will be positioned relative to the bottom
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [bottom], then the bottom
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  bottom,

  /// If the [WindowPositioner.parentAnchor] is set to [left], then the
  /// child [Window] will be positioned relative to the left
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [left], then the left
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  left,

  /// If the [WindowPositioner.parentAnchor] is set to [right], then the
  /// child [Window] will be positioned relative to the right
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [right], then the right
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  right,

  /// If the [WindowPositioner.parentAnchor] is set to [topLeft], then the
  /// child [Window] will be positioned relative to the top left
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [topLeft], then the top left
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  topLeft,

  /// If the [WindowPositioner.parentAnchor] is set to [bottomLeft], then the
  /// child [Window] will be positioned relative to the bottom left
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [bottomLeft], then the bottom left
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  bottomLeft,

  /// If the [WindowPositioner.parentAnchor] is set to [topRight], then the
  /// child [Window] will be positioned relative to the top right
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [topRight], then the top right
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  topRight,

  /// If the [WindowPositioner.parentAnchor] is set to [bottomRight], then the
  /// child [Window] will be positioned relative to the bottom right
  /// of the parent [Window].
  ///
  /// If [WindowPositioner.childAnchor] is set to  [bottomRight], then the bottom right
  /// of the child [Window] will be positioned relative to
  /// [WindowPositioner.parentAnchor].
  bottomRight,
}

/// The [WindowPositionerConstraintAdjustment] value defines the ways in which
/// Flutter will adjust the position of the [Window], if the unadjusted position would result
/// in the surface being partly constrained.
///
/// Whether a [Window] is considered 'constrained' is left to the platform
/// to determine. For example, the surface may be partly outside the
/// compositor's defined 'work area', thus necessitating the child [Window]'s
/// position be adjusted until it is entirely inside the work area.
///
/// 'Flip' means reverse the anchor points and offset along an axis.
/// 'Slide' means adjust the offset along an axis.
/// 'Resize' means adjust the client [Window] size along an axis.
///
/// The adjustments can be combined, according to a defined precedence: 1)
/// Flip, 2) Slide, 3) Resize.
enum WindowPositionerConstraintAdjustment {
  /// If [slideX] is specified in [WindowPositioner.constraintAdjustment]
  /// and the [Window] would be displayed off the screen in the X-axis, then it will be
  /// translated in the X-direction (either negative or positive) in order
  /// to best display the window on screen.
  slideX,

  /// If [slideY] is specified in [WindowPositioner.constraintAdjustment]
  /// and the [Window] would be displayed off the screen in the Y-axis, then it will be
  /// translated in the Y-direction (either negative or positive) in order
  /// to best display the window on screen.
  slideY,

  /// If [flipX] is specified in [WindowPositioner.constraintAdjustment]
  /// and the [Window] would be displayed off the screen in the X-axis in one direction, then
  /// it will be flipped to the opposite side of its parent in order to show
  /// to best display the window on screen.
  flipX,

  /// If [flipY] is specified in [WindowPositioner.constraintAdjustment]
  /// and then [Window] would be displayed off the screen in the Y-axis in one direction, then
  /// it will be flipped to the opposite side of its parent in order to show
  /// it on screen.
  flipY,

  /// If [resizeX] is specified in [WindowPositioner.constraintAdjustment]
  /// and the [Window] would be displayed off the screen in the X-axis, then
  /// its width will be reduced such that it fits on screen.
  resizeX,

  /// If [resizeY] is specified in [WindowPositioner.constraintAdjustment]
  /// and the [Window] would be displayed off the screen in the Y-axis, then
  /// its height will be reduced such that it fits on screen.
  resizeY,
}

/// The [WindowPositioner] provides a collection of rules for the placement
/// of a child [Window] relative to a parent [Window]. Rules can be defined to ensure
/// the child [Window] remains within the visible area's borders, and to
/// specify how the child [Window] changes its position, such as sliding along
/// an axis, or flipping around a rectangle.
class WindowPositioner {
  /// Const constructor for [WindowPositioner].
  const WindowPositioner({
    this.parentAnchor = WindowPositionerAnchor.center,
    this.childAnchor = WindowPositionerAnchor.center,
    this.offset = Offset.zero,
    this.constraintAdjustment = const <WindowPositionerConstraintAdjustment>{},
  });

  /// Copy a [WindowPositioner] with some fields replaced.
  WindowPositioner copyWith({
    WindowPositionerAnchor? parentAnchor,
    WindowPositionerAnchor? childAnchor,
    Offset? offset,
    Set<WindowPositionerConstraintAdjustment>? constraintAdjustment,
  }) {
    return WindowPositioner(
      parentAnchor: parentAnchor ?? this.parentAnchor,
      childAnchor: childAnchor ?? this.childAnchor,
      offset: offset ?? this.offset,
      constraintAdjustment: constraintAdjustment ?? this.constraintAdjustment,
    );
  }

  /// Defines the anchor point for the anchor rectangle. The specified anchor
  /// is used to derive an anchor point that the child [Window] will be
  /// positioned relative to. If a corner anchor is set (e.g. [topLeft] or
  /// [bottomRight]), the anchor point will be at the specified corner;
  /// otherwise, the derived anchor point will be centered on the specified
  /// edge, or in the center of the anchor rectangle if no edge is specified.
  final WindowPositionerAnchor parentAnchor;

  /// Defines the anchor point for the child [Window]. The specified anchor
  /// is used to derive an anchor point that will be positioned relative to the
  /// parentAnchor. If a corner anchor is set (e.g. [topLeft] or
  /// [bottomRight]), the anchor point will be at the specified corner;
  /// otherwise, the derived anchor point will be centered on the specified
  /// edge, or in the center of the anchor rectangle if no edge is specified.
  final WindowPositionerAnchor childAnchor;

  /// Specify the [Window] position offset relative to the position of the
  /// anchor on the anchor rectangle and the anchor on the child. For
  /// example if the anchor of the anchor rectangle is at (x, y), the [Window]
  /// has the child_anchor [topLeft], and the offset is (ox, oy), the calculated
  /// [Window] position will be (x + ox, y + oy). The offset position of the
  /// [Window] is the one used for constraint testing. See constraintAdjustment.
  ///
  /// An example use case is placing a popup menu on top of a user interface
  /// element, while aligning the user interface element of the parent [Window]
  /// with some user interface element placed somewhere in the popup [Window].
  final Offset offset;

  /// The constraintAdjustment value define ways Flutter will adjust
  /// the position of the [Window], if the unadjusted position would result
  /// in the surface being partly constrained.
  ///
  /// Whether a [Window] is considered 'constrained' is left to the platform
  /// to determine. For example, the surface may be partly outside the
  /// output's 'work area', thus necessitating the child [Window]'s
  /// position be adjusted until it is entirely inside the work area.
  ///
  /// The adjustments can be combined, according to a defined precedence: 1)
  /// Flip, 2) Slide, 3) Resize.
  final Set<WindowPositionerConstraintAdjustment> constraintAdjustment;
}

/// Controller used with the [RegularWindow] widget. This controller
/// provides access to modify and destroy the window, in addition to
/// listening to changes on the window.
abstract class WindowController with ChangeNotifier {
  FlutterView? _view;

  /// The ID of the view used for this window, which is unique to each window.
  FlutterView? get view => _view;
  set view(FlutterView? value) {
    _view = value;
    notifyListeners();
  }

  Size? _size;

  /// The current size of the window. This may differ from the requested size.
  Size? get size => _size;
  set size(Size? value) {
    _size = value;
    notifyListeners();
  }

  int? _parentViewId;

  /// The ID of the parent in which this rendered, if any.
  int? get parentViewId => _parentViewId;
  set parentViewId(int? value) {
    _parentViewId = value;
    notifyListeners();
  }

  /// The archetype of the window.
  WindowArchetype get type;

  /// Destroys this window.
  Future<void> destroy() async {
    if (view == null) {
      return;
    }

    return destroyWindow(view!.viewId);
  }
}

/// Provided to [RegularWindow]. Allows the user to listen on changes
/// to a regular window and modify the window.
class RegularWindowController extends WindowController {
  @override
  WindowArchetype get type => WindowArchetype.regular;

  /// Modify the properties of the window.
  Future<void> modify({Size? size}) {
    throw UnimplementedError();
  }
}

class PopupWindowController extends WindowController {
  @override
  WindowArchetype get type => WindowArchetype.popup;
}

class _GenericWindow extends StatefulWidget {
  _GenericWindow(
      {this.onDestroyed,
      this.onError,
      super.key,
      required this.createFuture,
      required this.controller,
      required this.child});

  final Future<WindowCreationResult> Function() createFuture;
  final WindowController? controller;
  final void Function()? onDestroyed;
  final void Function(String?)? onError;
  final Widget child;

  @override
  State<_GenericWindow> createState() => _GenericWindowState();
}

class _GenericWindowState extends State<_GenericWindow> {
  _WindowListener? _listener;
  Future<WindowCreationResult>? _future;
  _WindowingAppState? _app;
  int? _viewId;
  bool _hasBeenDestroyed = false;

  @override
  void initState() {
    super.initState();
    setState(() {
      _future = widget.createFuture();
    });

    _future!.then((WindowCreationResult metadata) async {
      _viewId = metadata.view.viewId;
      if (widget.controller != null) {
        widget.controller!.view = metadata.view;
        widget.controller!.parentViewId = metadata.parent;
        widget.controller!.size = metadata.size;
      }

      final _WindowingAppContext? windowingAppContext = _WindowingAppContext.of(context);
      assert(windowingAppContext != null);
      _listener = _WindowListener(
          viewId: metadata.view.viewId,
          onChanged: (_WindowChangeProperties properties) {
            if (widget.controller == null) {
              return;
            }

            if (properties.size != null) {
              widget.controller!.size = properties.size;
            }

            if (properties.parentViewId != null) {
              widget.controller!.parentViewId = properties.parentViewId;
            }
          },
          onDestroyed: () {
            widget.onDestroyed?.call();
            _hasBeenDestroyed = true;
          });
      _app = windowingAppContext!.windowingApp;
      _app!._registerListener(_listener!);
    }).catchError((Object? error) {
      print(error.toString());
      widget.onError?.call(error.toString());
    });
  }

  @override
  Future<void> dispose() async {
    if (_listener != null) {
      assert(_app != null);
      _app!._unregisterListener(_listener!);
    }

    // In the event that we're being disposed before we've been destroyed
    // we need to destroy the window on our way out.
    if (!_hasBeenDestroyed && _viewId != null) {
      // In the event of an argument error, we do nothing. We assume that
      // the window has been successfully destroyed somehow else.
      try {
        await destroyWindow(_viewId!);
      } on ArgumentError {}
    }

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<WindowCreationResult>(
        key: widget.key,
        future: _future,
        builder: (BuildContext context, AsyncSnapshot<WindowCreationResult> metadata) {
          if (!metadata.hasData) {
            return const ViewCollection(views: <Widget>[]);
          }

          return View(
              view: metadata.data!.view,
              child: WindowContext(view: metadata.data!.view, child: widget.child));
        });
  }
}

/// A widget that creates a regular window. This content of this window is
/// rendered into a [View], meaning that this widget must be rendered into
/// either a [ViewAnchor] or a [ViewCollection].
class RegularWindow extends StatelessWidget {
  /// Creates a regular window widget
  const RegularWindow(
      {this.controller,
      this.onDestroyed,
      this.onError,
      super.key,
      required this.preferredSize,
      required this.child});

  final RegularWindowController? controller;

  /// Called when the window backing this widget is destroyed.
  final void Function()? onDestroyed;

  /// Called when an error is encountered during the creation of this widget.
  final void Function(String?)? onError;

  /// Preferred size of the window.
  final Size preferredSize;

  /// The content rendered into this window.
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return _GenericWindow(
        onDestroyed: onDestroyed,
        onError: onError,
        key: key,
        createFuture: () => createRegular(size: preferredSize),
        controller: controller,
        child: child);
  }
}

/// A widget that creates a popup window. This content of this window is
/// rendered into a [View], meaning that this widget must be rendered into
/// either a [ViewAnchor] or a [ViewCollection].
class PopupWindow extends StatelessWidget {
  /// Creates a popup window widget
  const PopupWindow(
      {this.controller,
      this.onDestroyed,
      this.onError,
      super.key,
      required Size preferredSize,
      Rect? anchorRect,
      WindowPositioner positioner = const WindowPositioner(),
      required this.child})
      : _preferredSize = preferredSize,
        _anchorRect = anchorRect,
        _positioner = positioner;

  /// Controller for this widget.
  final PopupWindowController? controller;

  /// Called when the window backing this widget is destroyed.
  final void Function()? onDestroyed;

  /// Called when an error is encountered during the creation of this widget.
  final void Function(String?)? onError;

  final Size _preferredSize;

  final Rect? _anchorRect;

  final WindowPositioner _positioner;

  /// The content rendered into this window.
  final Widget child;

  Rect _clampRectToSize(BuildContext context, Rect anchorRect) {
    final double dpr = MediaQuery.of(context).devicePixelRatio;
    final WindowContext windowContext = WindowContext.of(context)!;
    final Size size = windowContext.view.physicalSize / dpr;

    final double left = anchorRect.left.clamp(0, size.width);
    final double top = anchorRect.top.clamp(0, size.height);
    final double right = anchorRect.right.clamp(0, size.width);
    final double bottom = anchorRect.bottom.clamp(0, size.height);
    return Rect.fromLTRB(left, top, right, bottom);
  }

  @override
  Widget build(BuildContext context) {
    final WindowContext? windowContext = WindowContext.of(context);
    assert(windowContext != null, 'A PopupWindow must have a parent');

    return _GenericWindow(
        onDestroyed: onDestroyed,
        onError: onError,
        key: key,
        createFuture: () => createPopup(
            parentViewId: windowContext!.viewId,
            size: _preferredSize,
            anchorRect: _anchorRect == null ? null : _clampRectToSize(context, _anchorRect),
            positioner: _positioner),
        controller: controller,
        child: child);
  }
}

/// Provides descendents with access to the [Window] in which they are rendered
class WindowContext extends InheritedWidget {
  /// [window] the [Window]
  const WindowContext({super.key, required this.view, required super.child});

  /// The view backing this window.
  final FlutterView view;

  /// The id of the current window.
  int get viewId => view.viewId;

  /// Returns the [WindowContext] if any
  static WindowContext? of(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<WindowContext>();
  }

  @override
  bool updateShouldNotify(WindowContext oldWidget) {
    return viewId != oldWidget.viewId;
  }
}

/// The raw data returned as a result of creating a window.
class WindowCreationResult {
  /// Creates a new window.
  WindowCreationResult(
      {required this.view, required this.archetype, required this.size, this.parent});

  /// The view associated with the window.
  final FlutterView view;

  /// The archetype of the window.
  final WindowArchetype archetype;

  /// The initial size of the window.
  final Size size;

  /// The id of the window's parent, if any.
  final int? parent;
}

/// Creates a regular window for the platform and returns the metadata associated
/// with the new window. Users should prefer using the [RegularWindow]
/// widget instead of this method.
///
/// [size] the size of the new [Window] in pixels
Future<WindowCreationResult> createRegular({required Size size}) {
  return _createWindow(viewBuilder: (MethodChannel channel) async {
    return await channel.invokeMethod('createWindow', <String, dynamic>{
      'size': <int>[size.width.toInt(), size.height.toInt()],
    }) as Map<Object?, Object?>;
  });
}

Future<WindowCreationResult> createPopup(
    {required int parentViewId,
    required Size size,
    Rect? anchorRect,
    required WindowPositioner positioner}) {
  int constraintAdjustmentBitmask = 0;
  for (final WindowPositionerConstraintAdjustment adjustment in positioner.constraintAdjustment) {
    constraintAdjustmentBitmask |= 1 << adjustment.index;
  }

  return _createWindow(viewBuilder: (MethodChannel channel) async {
    return await channel.invokeMethod('createPopup', <String, dynamic>{
      'parent': parentViewId,
      'size': <int>[size.width.toInt(), size.height.toInt()],
      'anchorRect': anchorRect != null
          ? <int>[
              anchorRect.left.toInt(),
              anchorRect.top.toInt(),
              anchorRect.width.toInt(),
              anchorRect.height.toInt()
            ]
          : null,
      'positionerParentAnchor': positioner.parentAnchor.index,
      'positionerChildAnchor': positioner.childAnchor.index,
      'positionerOffset': <int>[positioner.offset.dx.toInt(), positioner.offset.dy.toInt()],
      'positionerConstraintAdjustment': constraintAdjustmentBitmask
    }) as Map<Object?, Object?>;
  });
}

Future<WindowCreationResult> _createWindow(
    {required Future<Map<Object?, Object?>> Function(MethodChannel channel) viewBuilder}) async {
  WidgetsFlutterBinding.ensureInitialized();
  final Map<Object?, Object?> creationData = await viewBuilder(SystemChannels.windowing);
  final int viewId = creationData['viewId']! as int;
  final WindowArchetype archetype = WindowArchetype.values[creationData['archetype']! as int];
  final List<Object?> size = creationData['size']! as List<Object?>;
  final int? parentViewId = creationData['parentViewId'] as int?;

  final FlutterView flView = WidgetsBinding.instance.platformDispatcher.views.firstWhere(
    (FlutterView view) => view.viewId == viewId,
    orElse: () {
      throw Exception('No matching view found for viewId: $viewId');
    },
  );

  return WindowCreationResult(
      view: flView,
      archetype: archetype,
      size: Size((size[0]! as int).toDouble(), (size[1]! as int).toDouble()),
      parent: parentViewId);
}

/// Destroys the window associated with the provided view ID.
///
/// [viewId] the view id of the window that should be destroyed
Future<void> destroyWindow(int viewId) async {
  try {
    await SystemChannels.windowing
        .invokeMethod('destroyWindow', <String, dynamic>{'viewId': viewId});
  } on PlatformException catch (e) {
    throw ArgumentError(
        'Unable to delete window with view_id=$viewId. Does the window exist? Error: $e');
  }
}

class _WindowChangeProperties {
  _WindowChangeProperties({this.size, this.parentViewId});

  Size? size;
  int? parentViewId;
}

class _WindowListener {
  _WindowListener({required this.viewId, required this.onChanged, required this.onDestroyed});

  int viewId;
  void Function(_WindowChangeProperties) onChanged;
  void Function()? onDestroyed;
}

/// Declares that an application will create multiple windows.
class WindowingApp extends StatefulWidget {
  /// Creates a new windowing app with the provided child windows.
  const WindowingApp({super.key, required this.children});

  /// A list of initial windows to render. These windows will be placed inside
  /// of a [ViewCollection].
  final List<Widget> children;

  @override
  State<WindowingApp> createState() => _WindowingAppState();
}

class _WindowingAppState extends State<WindowingApp> {
  final List<_WindowListener> _listeners = <_WindowListener>[];

  @override
  void initState() {
    super.initState();
    WidgetsFlutterBinding.ensureInitialized();
    SystemChannels.windowing.setMethodCallHandler(_methodCallHandler);
  }

  Future<void> _methodCallHandler(MethodCall call) async {
    final Map<Object?, Object?> arguments = call.arguments as Map<Object?, Object?>;

    switch (call.method) {
      case 'onWindowCreated':
        final int viewId = arguments['viewId']! as int;
        int? parentViewId;
        if (arguments['parentViewId'] != null) {
          parentViewId = arguments['parentViewId']! as int;
        }

        final _WindowChangeProperties properties =
            _WindowChangeProperties(parentViewId: parentViewId);
        for (final _WindowListener listener in _listeners) {
          if (listener.viewId == viewId) {
            listener.onChanged(properties);
          }
        }
      case 'onWindowChanged':
        final int viewId = arguments['viewId']! as int;
        Size? size;
        if (arguments['size'] != null) {
          final List<Object?> sizeRaw = arguments['size']! as List<Object?>;
          size = Size((sizeRaw[0]! as int).toDouble(), (sizeRaw[1]! as int).toDouble());
        }

        final _WindowChangeProperties properties = _WindowChangeProperties(size: size);
        for (final _WindowListener listener in _listeners) {
          if (listener.viewId == viewId) {
            listener.onChanged(properties);
          }
        }
      case 'onWindowDestroyed':
        final int viewId = arguments['viewId']! as int;
        for (final _WindowListener listener in _listeners) {
          if (listener.viewId == viewId) {
            listener.onDestroyed?.call();
          }
        }
    }
  }

  void _registerListener(_WindowListener listener) {
    _listeners.add(listener);
  }

  void _unregisterListener(_WindowListener listener) {
    _listeners.remove(listener);
  }

  @override
  Widget build(BuildContext context) {
    return _WindowingAppContext(windowingApp: this, child: ViewCollection(views: widget.children));
  }
}

class _WindowingAppContext extends InheritedWidget {
  const _WindowingAppContext({super.key, required super.child, required this.windowingApp});

  final _WindowingAppState windowingApp;

  /// Returns the [MultiWindowAppContext] if any
  static _WindowingAppContext? of(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<_WindowingAppContext>();
  }

  @override
  bool updateShouldNotify(_WindowingAppContext oldWidget) {
    return windowingApp != oldWidget.windowingApp;
  }
}
