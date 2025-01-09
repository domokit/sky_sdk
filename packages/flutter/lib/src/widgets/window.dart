// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show FlutterView;

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

/// Defines the type of the Window
enum WindowArchetype {
  /// Defines a traditional window
  regular
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

/// A widget that creates a regular window. This content of this window is
/// rendered into a [View], meaning that this widget must be rendered into
/// either a [ViewAnchor] or a [ViewCollection].
class RegularWindow extends StatefulWidget {
  /// Creates a regular window widget
  const RegularWindow(
      {this.controller,
      this.onDestroyed,
      this.onError,
      super.key,
      required this.preferredSize,
      required this.child});

  /// Controller for this widget.
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
  State<RegularWindow> createState() => _RegularWindowState();
}

class _RegularWindowState extends State<RegularWindow> {
  _WindowListener? _listener;
  Future<WindowCreationResult>? _future;
  _WindowingAppState? _app;
  int? _viewId;
  bool _hasBeenDestroyed = false;

  @override
  void initState() {
    super.initState();
    final Future<WindowCreationResult> createRegularFuture =
        createRegular(size: widget.preferredSize);
    setState(() {
      _future = createRegularFuture;
    });

    createRegularFuture.then((WindowCreationResult metadata) async {
      _viewId = metadata.view.viewId;
      if (widget.controller != null) {
        widget.controller!.view = metadata.view;
        widget.controller!.parentViewId = metadata.parent;
        widget.controller!.size = metadata.size;
      }

      SchedulerBinding.instance.addPostFrameCallback((_) async {
        final _WindowingAppContext? windowingAppContext =
            _WindowingAppContext.of(context);
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
      });
    }).catchError((Object? error) {
      widget.onError?.call(error.toString());
    });
  }

  @override
  Future<void> dispose() async {
    super.dispose();

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
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<WindowCreationResult>(
        key: widget.key,
        future: _future,
        builder: (BuildContext context,
            AsyncSnapshot<WindowCreationResult> metadata) {
          if (!metadata.hasData) {
            return const ViewCollection(views: <Widget>[]);
          }

          return View(
              view: metadata.data!.view,
              child: WindowContext(
                  viewId: metadata.data!.view.viewId, child: widget.child));
        });
  }
}

/// Provides descendents with access to the [Window] in which they are rendered
class WindowContext extends InheritedWidget {
  /// [window] the [Window]
  const WindowContext({super.key, required this.viewId, required super.child});

  /// The view ID in this context
  final int viewId;

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
      {required this.view,
      required this.archetype,
      required this.size,
      this.parent});

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

Future<WindowCreationResult> _createWindow(
    {required Future<Map<Object?, Object?>> Function(MethodChannel channel)
        viewBuilder}) async {
  WidgetsFlutterBinding.ensureInitialized();
  final Map<Object?, Object?> creationData =
      await viewBuilder(SystemChannels.windowing);
  final int viewId = creationData['viewId']! as int;
  final WindowArchetype archetype =
      WindowArchetype.values[creationData['archetype']! as int];
  final List<Object?> size = creationData['size']! as List<Object?>;
  final int? parentViewId = creationData['parentViewId'] as int?;

  final FlutterView flView =
      WidgetsBinding.instance.platformDispatcher.views.firstWhere(
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
  _WindowListener(
      {required this.viewId,
      required this.onChanged,
      required this.onDestroyed});

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
    final Map<Object?, Object?> arguments =
        call.arguments as Map<Object?, Object?>;

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
          size = Size(
              (sizeRaw[0]! as int).toDouble(), (sizeRaw[1]! as int).toDouble());
        }

        final _WindowChangeProperties properties =
            _WindowChangeProperties(size: size);
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
    return _WindowingAppContext(
        windowingApp: this, child: ViewCollection(views: widget.children));
  }
}

class _WindowingAppContext extends InheritedWidget {
  const _WindowingAppContext(
      {super.key, required super.child, required this.windowingApp});

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
