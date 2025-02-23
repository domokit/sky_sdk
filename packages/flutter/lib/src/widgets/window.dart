// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show FlutterView;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Defines the possible archetypes for a window.
enum WindowArchetype {
  /// Defines a traditional window
  regular,
}

/// Defines the possible states that a window can be in.
enum WindowState {
  /// Window is in its normal state, neither maximized, nor minimized.
  restored,

  /// Window is maximized, occupying the full screen but still showing the system UI.
  maximized,

  /// Window is minimized and not visible on the screen.
  minimized,
}

/// Base class for window controllers.
///
/// A window controller must provide a [future] that resolves to a
/// a [WindowCreationResult] object. This object contains the view
/// associated with the window, the archetype of the window, the size
/// of the window, and the state of the window.
///
/// The caller may also provide a callback to be called when the window
/// is destroyed, and a callback to be called when an error is encountered
/// during the creation of the window.
///
/// Each [WindowController] is associated with exactly one root [FlutterView].
///
/// When the window is destroyed for any reason (either by the caller or by the
/// platform), the content of the controller will thereafter be invalid. Callers
/// may check if this content is invalid via the [isReady] property.
///
/// This class implements the [Listenable] interface, so callers can listen
/// for changes to the window's properties.
abstract class WindowController with ChangeNotifier {
  /// Creates a [WindowController] with the provided properties.
  /// Upon construction, this widget begins creating a window for the platform.
  /// The [future] parameter is a future that resolves to a [WindowCreationResult]
  /// object.
  /// The [onDestroyed] parameter is a callback that is called when the window
  /// is destroyed.
  /// The [onError] parameter is a callback that is called when an error is
  /// encountered during creation.
  WindowController({
    VoidCallback? onDestroyed,
    void Function(String)? onError,
    required this.future,
  }) {
    future
        .then((WindowCreationResult metadata) async {
          _handleCreationResult(metadata);

          _listener = _WindowListener(
            viewId: metadata.rootView.viewId,
            onChanged: (_WindowChangeProperties properties) {
              if (properties.size != null) {
                _size = properties.size!;
                notifyListeners();
              }
            },
            onDestroyed: () {
              _view = null;
              _isPendingDestroy = false;
              notifyListeners();
              onDestroyed?.call();
            },
          );
          _WindowingAppGlobalData.instance._listen(_listener);
        })
        .catchError((Object? error) {
          onError?.call(error.toString());
        });
  }

  void _handleCreationResult(WindowCreationResult metadata) {
    _view = metadata.rootView;
    _size = metadata.size;
    notifyListeners();
  }

  /// Returns true if the window associated with the controller has been
  /// created and is ready to be used. Otherwise, returns false.
  bool get isReady => _view != null;

  /// The future that resolves when the window has been created.
  final Future<WindowCreationResult> future;

  late final _WindowListener _listener;

  /// The root view associated to this window, which is unique to each window.
  FlutterView get rootView => _view!;
  FlutterView? _view;

  /// The current size of the window. This may differ from the requested size.
  Size get size => _size;
  Size _size = Size.zero;

  /// The archetype of the window.
  WindowArchetype get type;

  bool _isPendingDestroy = false;

  /// Destroys this window. If the window is not ready or is already pending
  /// destruction, then nothing happens. Otherwise, we begin destroying the
  /// window.
  Future<void> destroy() async {
    if (!isReady || _isPendingDestroy) {
      return;
    }

    _isPendingDestroy = true;
    return _destroyWindow(rootView.viewId);
  }
}

/// A controller for a regular window.
///
/// A regular window is a traditional window that can be resized, minimized,
/// maximized, and closed. Upon construction, the window is created for the
/// platform with the provided properties.
///
/// This class does not interact with the widget tree. Instead, it is typically
/// provided to the [RegularWindow] widget, who does the work of rendering the
/// content inside of this window.
///
/// An example usage might look like:
/// ```dart
/// final RegularWindowController controller = RegularWindowController(
///   size: const Size(800, 600),
///   sizeConstraints: const BoxConstraints(minWidth: 640, minHeight: 480),
///   title: "Example Window",
/// );
/// runWidget(RegularWindow(
///   controller: controller,
///   child: MaterialApp(home: Container())));
/// ```
///
/// When provided to a [RegularWindow] widget, widgets inside of the [child]
/// parameter will have access to the [RegularWindowController] via the
/// [WindowControllerContext] widget.
class RegularWindowController extends WindowController {
  /// Creates a [RegularWindowController] with the provided properties.
  /// Upon construction, the window is created for the platform.
  ///
  /// [title] the title of the window
  /// [state] the initial state of the window
  /// [sizeConstraints] the size constraints of the window
  /// [onDestroyed] a callback that is called when the window is destroyed
  /// [onError] a callback that is called when an error is encountered
  /// [size] the size of the window
  RegularWindowController({
    String? title,
    WindowState? state,
    BoxConstraints? sizeConstraints,
    VoidCallback? onDestroyed,
    void Function(String)? onError,
    required Size size,
  }) : super(
         onDestroyed: onDestroyed,
         onError: onError,
         future: _createRegular(
           size: size,
           sizeConstraints: sizeConstraints,
           title: title,
           state: state,
         ),
       ) {}

  @override
  void _handleCreationResult(WindowCreationResult metadata) {
    _state = (metadata as _RegularWindowCreationResult).state;
    super._handleCreationResult(metadata);
  }

  @override
  WindowArchetype get type => WindowArchetype.regular;

  /// The current state of the window.
  WindowState get state => _state;
  WindowState _state = WindowState.restored;

  /// Modify the properties of the window. The window must be ready before
  /// calling this method. If the window is not ready, an assertion will be
  /// thrown. The caller must provide at least one of the following parameters:
  ///
  /// [size] the new size of the window
  /// [title] the new title of the window
  /// [state] the new state of the window
  ///
  /// If no parameters are provided, then an assertion will be thrown.
  Future<void> modify({Size? size, String? title, WindowState? state}) {
    assert(isReady, 'Window is not ready');
    return _modifyRegular(viewId: rootView.viewId, size: size, title: title, state: state);
  }
}

/// The [RegularWindow] widget provides a way to render a regular window in the
/// widget tree. The provided [controller] creates the native window that backs
/// the widget. The [child] widget is rendered into this newly created window.
///
/// While the window is being created, the [RegularWindow] widget will render
/// an empty [ViewCollection] widget. Once the window is created, the [child]
/// widget will be rendered into the window inside of a [View].
///
/// An example usage might look like:
/// ```dart
/// final RegularWindowController controller = RegularWindowController(
///   size: const Size(800, 600),
///   sizeConstraints: const BoxConstraints(minWidth: 640, minHeight: 480),
///   title: "Example Window",
/// );
/// runApp(RegularWindow(
///   controller: controller,
///   child: MaterialApp(home: Container())));
/// ```
///
/// When a [RegularWindow] widget is removed from the tree, the window that was created
/// by the [controller] is automatically destroyed if it has not yet been destroyed.
///
/// Widgets in the same tree as the [child] widget will have access to the
/// [RegularWindowController] via the [WindowControllerContext] widget.
class RegularWindow extends StatefulWidget {
  /// Creates a regular window widget.
  /// [controller] the controller for this window
  /// [child] the content to render into this window
  /// [key] the key for this widget
  const RegularWindow({super.key, required this.controller, required this.child});

  /// Controller for this widget.
  final RegularWindowController controller;

  /// The content rendered into this window.
  final Widget child;

  @override
  State<RegularWindow> createState() => _RegularWindowState();
}

class _RegularWindowState extends State<RegularWindow> {
  @override
  Future<void> dispose() async {
    super.dispose();
    await widget.controller.destroy();
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<WindowCreationResult>(
      key: widget.key,
      future: widget.controller.future,
      builder: (BuildContext context, AsyncSnapshot<WindowCreationResult> metadata) {
        if (!metadata.hasData) {
          return const ViewCollection(views: <Widget>[]);
        }

        return View(
          view: metadata.data!.rootView,
          child: WindowControllerContext(controller: widget.controller, child: widget.child),
        );
      },
    );
  }
}

/// Provides descendents with access to the [WindowController] associated with
/// the window that is being rendered.
class WindowControllerContext extends InheritedWidget {
  /// Creates a new [WindowControllerContext]
  /// [controller] the controller associated with this window
  /// [child] the child widget
  const WindowControllerContext({super.key, required this.controller, required super.child});

  /// The controller associated with this window.
  final WindowController controller;

  /// Returns the [WindowContext] if any
  static WindowController? of(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<WindowControllerContext>()?.controller;
  }

  @override
  bool updateShouldNotify(WindowControllerContext oldWidget) {
    return controller != oldWidget.controller;
  }
}

/// Used by the [WindowController], this class defines the raw data
/// associated with the creation of a window. This object can be constructed
/// from anywhere, but it is typically returned by internal methods that create
/// windows.
abstract class WindowCreationResult {
  /// Creates a new [WindowCreationResult]
  /// [rootView] the view associated with this window
  /// [archetype] the archetype of this window
  /// [size] the size of this window
  /// [state] the state of this window
  WindowCreationResult({
    this.parentView,
    required this.rootView,
    required this.archetype,
    required this.size,
  });

  /// The view associated with this window.
  final FlutterView rootView;

  final FlutterView? parentView;

  /// The archetype of this window.
  final WindowArchetype archetype;

  /// The size of this window.
  final Size size;
}

class _RegularWindowCreationResult extends WindowCreationResult {
  _RegularWindowCreationResult({
    super.parentView,
    required super.rootView,
    required super.archetype,
    required super.size,
    required this.state,
  });

  /// The state of the window
  final WindowState state;
}

Future<_RegularWindowCreationResult> _createRegular({
  required Size size,
  BoxConstraints? sizeConstraints,
  String? title,
  WindowState? state,
}) async {
  WidgetsFlutterBinding.ensureInitialized();
  final Map<Object?, Object?> creationData =
      await SystemChannels.windowing.invokeMethod('createRegular', <String, dynamic>{
            'size': <double>[size.width, size.height],
            'minSize':
                sizeConstraints != null
                    ? <double>[sizeConstraints.minWidth, sizeConstraints.minHeight]
                    : null,
            'maxSize':
                sizeConstraints != null
                    ? <double>[sizeConstraints.maxWidth, sizeConstraints.maxHeight]
                    : null,
            'title': title,
            'state': state?.toString(),
          })
          as Map<Object?, Object?>;
  final int viewId = creationData['viewId']! as int;
  final List<Object?> resultSize = creationData['size']! as List<Object?>;
  final WindowState resultState =
      (creationData['state'] as String?) != null
          ? WindowState.values.firstWhere(
            (WindowState e) => e.toString() == creationData['state'],
            orElse:
                () => throw Exception('Invalid window state received: ${creationData['state']}'),
          )
          : WindowState.restored;

  final FlutterView flView = WidgetsBinding.instance.platformDispatcher.views.firstWhere(
    (FlutterView view) => view.viewId == viewId,
    orElse: () {
      throw Exception('No matching view found for viewId: $viewId');
    },
  );

  return _RegularWindowCreationResult(
    rootView: flView,
    archetype: WindowArchetype.regular,
    size: Size(resultSize[0]! as double, resultSize[1]! as double),
    state: resultState,
  );
}

Future<void> _modifyRegular({required int viewId, Size? size, String? title, WindowState? state}) {
  assert(size != null || title != null || state != null);
  return SystemChannels.windowing.invokeMethod('modifyRegular', <String, dynamic>{
    'viewId': viewId,
    'size': size != null ? <double>[size.width, size.height] : null,
    'title': title,
    'state': state?.toString(),
  });
}

Future<void> _destroyWindow(int viewId) async {
  try {
    await SystemChannels.windowing.invokeMethod('destroyWindow', <String, dynamic>{
      'viewId': viewId,
    });
  } on PlatformException catch (e) {
    throw ArgumentError(
      'Unable to delete window with view_id=$viewId. Does the window exist? Error: $e',
    );
  }
}

class _WindowChangeProperties {
  _WindowChangeProperties({this.size});

  Size? size;
  int? parentViewId;
}

class _WindowListener {
  _WindowListener({required this.viewId, required this.onChanged, required this.onDestroyed});

  int viewId;
  void Function(_WindowChangeProperties) onChanged;
  void Function()? onDestroyed;
}

class _WindowingAppGlobalData {
  _WindowingAppGlobalData() {
    WidgetsFlutterBinding.ensureInitialized();
    SystemChannels.windowing.setMethodCallHandler(_methodCallHandler);
  }

  static _WindowingAppGlobalData get instance {
    _instance ??= _WindowingAppGlobalData();
    return _instance!;
  }

  final List<_WindowListener> _listeners = <_WindowListener>[];
  static _WindowingAppGlobalData? _instance;

  Future<void> _methodCallHandler(MethodCall call) async {
    final Map<Object?, Object?> arguments = call.arguments as Map<Object?, Object?>;

    switch (call.method) {
      case 'onWindowChanged':
        final int viewId = arguments['viewId']! as int;
        Size? size;
        if (arguments['size'] != null) {
          final List<Object?> sizeRaw = arguments['size']! as List<Object?>;
          size = Size(sizeRaw[0]! as double, sizeRaw[1]! as double);
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

        _listeners.removeWhere((_WindowListener listener) => listener.viewId == viewId);
    }
  }

  void _listen(_WindowListener listener) {
    _listeners.add(listener);
  }
}
