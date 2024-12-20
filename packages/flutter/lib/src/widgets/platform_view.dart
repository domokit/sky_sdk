// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'gesture_detector.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import '_html_element_view_io.dart' if (dart.library.js_util) '_html_element_view_web.dart';
import 'basic.dart';
import 'debug.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'framework.dart';

// Examples can assume:
// PlatformViewController createFooWebView(PlatformViewCreationParams params) { return (null as dynamic) as PlatformViewController; }
// Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers = <Factory<OneSequenceGestureRecognizer>>{};
// late PlatformViewController _controller;
// void myOnElementCreated(Object element) {}
// void myOnPlatformViewCreated(int viewId) {}

/// Embeds an Android view in the Widget hierarchy.
///
/// Requires Android API level 23 or greater.
///
/// Embedding Android views is an expensive operation and should be avoided when a Flutter
/// equivalent is possible.
///
/// The embedded Android view is painted just like any other Flutter widget and transformations
/// apply to it as well.
///
/// {@template flutter.widgets.AndroidView.layout}
/// The widget fills all available space, the parent of this object must provide bounded layout
/// constraints.
/// {@endtemplate}
///
/// {@template flutter.widgets.AndroidView.gestures}
/// The widget participates in Flutter's gesture arenas, and dispatches touch events to the
/// platform view iff it won the arena. Specific gestures that should be dispatched to the platform
/// view can be specified in the `gestureRecognizers` constructor parameter. If
/// the set of gesture recognizers is empty, a gesture will be dispatched to the platform
/// view iff it was not claimed by any other gesture recognizer.
/// {@endtemplate}
///
/// The Android view object is created using a [PlatformViewFactory](/javadoc/io/flutter/plugin/platform/PlatformViewFactory.html).
/// Plugins can register platform view factories with [PlatformViewRegistry#registerViewFactory](/javadoc/io/flutter/plugin/platform/PlatformViewRegistry.html#registerViewFactory-java.lang.String-io.flutter.plugin.platform.PlatformViewFactory-).
///
/// Registration is typically done in the plugin's registerWith method, e.g:
///
/// ```java
///   public static void registerWith(Registrar registrar) {
///     registrar.platformViewRegistry().registerViewFactory("webview", WebViewFactory(registrar.messenger()));
///   }
/// ```
///
/// {@template flutter.widgets.AndroidView.lifetime}
/// The platform view's lifetime is the same as the lifetime of the [State] object for this widget.
/// When the [State] is disposed the platform view (and auxiliary resources) are lazily
/// released (some resources are immediately released and some by platform garbage collector).
/// A stateful widget's state is disposed when the widget is removed from the tree or when it is
/// moved within the tree. If the stateful widget has a key and it's only moved relative to its siblings,
/// or it has a [GlobalKey] and it's moved within the tree, it will not be disposed.
/// {@endtemplate}
class AndroidView extends StatefulWidget {
  /// Creates a widget that embeds an Android view.
  ///
  /// {@template flutter.widgets.AndroidView.constructorArgs}
  /// If `creationParams` is not null then `creationParamsCodec` must not be null.
  /// {@endtemplate}
  const AndroidView({
    super.key,
    required this.viewType,
    this.onPlatformViewCreated,
    this.hitTestBehavior = PlatformViewHitTestBehavior.opaque,
    this.layoutDirection,
    this.gestureRecognizers,
    this.creationParams,
    this.creationParamsCodec,
    this.clipBehavior = Clip.hardEdge,
  }) : assert(creationParams == null || creationParamsCodec != null);

  /// The unique identifier for Android view type to be embedded by this widget.
  ///
  /// A [PlatformViewFactory](/javadoc/io/flutter/plugin/platform/PlatformViewFactory.html)
  /// for this type must have been registered.
  ///
  /// See also:
  ///
  ///  * [AndroidView] for an example of registering a platform view factory.
  final String viewType;

  /// {@template flutter.widgets.AndroidView.onPlatformViewCreated}
  /// Callback to invoke after the platform view has been created.
  ///
  /// May be null.
  /// {@endtemplate}
  final PlatformViewCreatedCallback? onPlatformViewCreated;

  /// {@template flutter.widgets.AndroidView.hitTestBehavior}
  /// How this widget should behave during hit testing.
  ///
  /// This defaults to [PlatformViewHitTestBehavior.opaque].
  /// {@endtemplate}
  final PlatformViewHitTestBehavior hitTestBehavior;

  /// {@template flutter.widgets.AndroidView.layoutDirection}
  /// The text direction to use for the embedded view.
  ///
  /// If this is null, the ambient [Directionality] is used instead.
  /// {@endtemplate}
  final TextDirection? layoutDirection;

  /// Which gestures should be forwarded to the Android view.
  ///
  /// {@template flutter.widgets.AndroidView.gestureRecognizers.descHead}
  /// The gesture recognizers built by factories in this set participate in the gesture arena for
  /// each pointer that was put down on the widget. If any of these recognizers win the
  /// gesture arena, the entire pointer event sequence starting from the pointer down event
  /// will be dispatched to the platform view.
  ///
  /// When null, an empty set of gesture recognizer factories is used, in which case a pointer event sequence
  /// will only be dispatched to the platform view if no other member of the arena claimed it.
  /// {@endtemplate}
  ///
  /// For example, with the following setup vertical drags will not be dispatched to the Android
  /// view as the vertical drag gesture is claimed by the parent [GestureDetector].
  ///
  /// ```dart
  /// GestureDetector(
  ///   onVerticalDragStart: (DragStartDetails d) {},
  ///   child: const AndroidView(
  ///     viewType: 'webview',
  ///   ),
  /// )
  /// ```
  ///
  /// To get the [AndroidView] to claim the vertical drag gestures we can pass a vertical drag
  /// gesture recognizer factory in [gestureRecognizers] e.g:
  ///
  /// ```dart
  /// GestureDetector(
  ///   onVerticalDragStart: (DragStartDetails details) {},
  ///   child: SizedBox(
  ///     width: 200.0,
  ///     height: 100.0,
  ///     child: AndroidView(
  ///       viewType: 'webview',
  ///       gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
  ///         Factory<OneSequenceGestureRecognizer>(
  ///           () => EagerGestureRecognizer(),
  ///         ),
  ///       },
  ///     ),
  ///   ),
  /// )
  /// ```
  ///
  /// {@template flutter.widgets.AndroidView.gestureRecognizers.descFoot}
  /// A platform view can be configured to consume all pointers that were put
  /// down in its bounds by passing a factory for an [EagerGestureRecognizer] in
  /// [gestureRecognizers]. [EagerGestureRecognizer] is a special gesture
  /// recognizer that immediately claims the gesture after a pointer down event.
  ///
  /// The [gestureRecognizers] property must not contain more than one factory
  /// with the same [Factory.type].
  ///
  /// Changing [gestureRecognizers] results in rejection of any active gesture
  /// arenas (if the platform view is actively participating in an arena).
  /// {@endtemplate}
  // We use OneSequenceGestureRecognizers as they support gesture arena teams.
  // TODO(amirh): get a list of GestureRecognizers here.
  // https://github.com/flutter/flutter/issues/20953
  final Set<Factory<OneSequenceGestureRecognizer>>? gestureRecognizers;

  /// Passed as the args argument of [PlatformViewFactory#create](/javadoc/io/flutter/plugin/platform/PlatformViewFactory.html#create-android.content.Context-int-java.lang.Object-)
  ///
  /// This can be used by plugins to pass constructor parameters to the embedded Android view.
  final dynamic creationParams;

  /// The codec used to encode `creationParams` before sending it to the
  /// platform side. It should match the codec passed to the constructor of [PlatformViewFactory](/javadoc/io/flutter/plugin/platform/PlatformViewFactory.html#PlatformViewFactory-io.flutter.plugin.common.MessageCodec-).
  ///
  /// This is typically one of: [StandardMessageCodec], [JSONMessageCodec], [StringCodec], or [BinaryCodec].
  ///
  /// This must not be null if [creationParams] is not null.
  final MessageCodec<dynamic>? creationParamsCodec;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  @override
  State<AndroidView> createState() => _AndroidViewState();
}

/// Common superclass for iOS and macOS platform views.
///
/// Platform views are used to embed native views in the widget hierarchy, with
/// support for transforms, clips, and opacity similar to any other Flutter widget.
abstract class _DarwinView extends StatefulWidget {
  /// Creates a widget that embeds a platform view.
  ///
  /// {@macro flutter.widgets.AndroidView.constructorArgs}
  const _DarwinView({
    super.key,
    required this.viewType,
    this.onPlatformViewCreated,
    this.hitTestBehavior = PlatformViewHitTestBehavior.opaque,
    this.layoutDirection,
    this.creationParams,
    this.creationParamsCodec,
    this.gestureRecognizers,
  }) : assert(creationParams == null || creationParamsCodec != null);

  // TODO(amirh): reference the iOS API doc once available.
  /// The unique identifier for iOS view type to be embedded by this widget.
  ///
  /// A PlatformViewFactory for this type must have been registered.
  final String viewType;

  /// {@macro flutter.widgets.AndroidView.onPlatformViewCreated}
  final PlatformViewCreatedCallback? onPlatformViewCreated;

  /// {@macro flutter.widgets.AndroidView.hitTestBehavior}
  final PlatformViewHitTestBehavior hitTestBehavior;

  /// {@macro flutter.widgets.AndroidView.layoutDirection}
  final TextDirection? layoutDirection;

  /// Passed as the `arguments` argument of [-\[FlutterPlatformViewFactory createWithFrame:viewIdentifier:arguments:\]](/ios-embedder/protocol_flutter_platform_view_factory-p.html#a4e3c4390cd6ebd982390635e9bca4edc)
  ///
  /// This can be used by plugins to pass constructor parameters to the embedded iOS view.
  final dynamic creationParams;

  /// The codec used to encode `creationParams` before sending it to the
  /// platform side. It should match the codec returned by [-\[FlutterPlatformViewFactory createArgsCodec:\]](/ios-embedder/protocol_flutter_platform_view_factory-p.html#a32c3c067cb45a83dfa720c74a0d5c93c)
  ///
  /// This is typically one of: [StandardMessageCodec], [JSONMessageCodec], [StringCodec], or [BinaryCodec].
  ///
  /// This must not be null if [creationParams] is not null.
  final MessageCodec<dynamic>? creationParamsCodec;

  /// Which gestures should be forwarded to the UIKit view.
  ///
  /// {@macro flutter.widgets.AndroidView.gestureRecognizers.descHead}
  ///
  /// For example, with the following setup vertical drags will not be dispatched to the UIKit
  /// view as the vertical drag gesture is claimed by the parent [GestureDetector].
  ///
  /// ```dart
  /// GestureDetector(
  ///   onVerticalDragStart: (DragStartDetails details) {},
  ///   child: const UiKitView(
  ///     viewType: 'webview',
  ///   ),
  /// )
  /// ```
  ///
  /// To get the [UiKitView] to claim the vertical drag gestures we can pass a vertical drag
  /// gesture recognizer factory in [gestureRecognizers] e.g:
  ///
  /// ```dart
  /// GestureDetector(
  ///   onVerticalDragStart: (DragStartDetails details) {},
  ///   child: SizedBox(
  ///     width: 200.0,
  ///     height: 100.0,
  ///     child: UiKitView(
  ///       viewType: 'webview',
  ///       gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
  ///         Factory<OneSequenceGestureRecognizer>(
  ///           () => EagerGestureRecognizer(),
  ///         ),
  ///       },
  ///     ),
  ///   ),
  /// )
  /// ```
  ///
  /// {@macro flutter.widgets.AndroidView.gestureRecognizers.descFoot}
  // We use OneSequenceGestureRecognizers as they support gesture arena teams.
  // TODO(amirh): get a list of GestureRecognizers here.
  // https://github.com/flutter/flutter/issues/20953
  final Set<Factory<OneSequenceGestureRecognizer>>? gestureRecognizers;
}

// TODO(amirh): describe the embedding mechanism.
// TODO(ychris): remove the documentation for conic path not supported once https://github.com/flutter/flutter/issues/35062 is resolved.
/// Embeds an iOS view in the Widget hierarchy.
///
/// Embedding iOS views is an expensive operation and should be avoided when a Flutter
/// equivalent is possible.
///
/// {@macro flutter.widgets.AndroidView.layout}
///
/// {@macro flutter.widgets.AndroidView.gestures}
///
/// {@macro flutter.widgets.AndroidView.lifetime}
///
/// Construction of UIViews is done asynchronously, before the UIView is ready this widget paints
/// nothing while maintaining the same layout constraints.
///
/// Clipping operations on a UiKitView can result slow performance.
/// If a conic path clipping is applied to a UIKitView,
/// a quad path is used to approximate the clip due to limitation of Quartz.
class UiKitView extends _DarwinView {
  /// Creates a widget that embeds an iOS view.
  ///
  /// {@macro flutter.widgets.AndroidView.constructorArgs}
  const UiKitView({
    super.key,
    required super.viewType,
    super.onPlatformViewCreated,
    super.hitTestBehavior = PlatformViewHitTestBehavior.opaque,
    super.layoutDirection,
    super.creationParams,
    super.creationParamsCodec,
    super.gestureRecognizers,
  }) : assert(creationParams == null || creationParamsCodec != null);

  @override
  State<UiKitView> createState() => _UiKitViewState();
}

/// Widget that contains a macOS AppKit view.
///
/// Embedding macOS views is an expensive operation and should be avoided where
/// a Flutter equivalent is possible.
///
/// The platform view's lifetime is the same as the lifetime of the [State]
/// object for this widget. When the [State] is disposed the platform view (and
/// auxiliary resources) are lazily released (some resources are immediately
/// released and some by platform garbage collector). A stateful widget's state
/// is disposed when the widget is removed from the tree or when it is moved
/// within the tree. If the stateful widget has a key and it's only moved
/// relative to its siblings, or it has a [GlobalKey] and it's moved within the
/// tree, it will not be disposed.
///
/// Construction of AppKitViews is done asynchronously, before the underlying
/// NSView is ready this widget paints nothing while maintaining the same
/// layout constraints.
class AppKitView extends _DarwinView {
  /// Creates a widget that embeds a macOS AppKit NSView.
  const AppKitView({
    super.key,
    required super.viewType,
    super.onPlatformViewCreated,
    super.hitTestBehavior = PlatformViewHitTestBehavior.opaque,
    super.layoutDirection,
    super.creationParams,
    super.creationParamsCodec,
    super.gestureRecognizers,
  });

  @override
  State<AppKitView> createState() => _AppKitViewState();
}

/// The signature of the function that gets called when the [HtmlElementView]
/// DOM element is created.
///
/// [element] is the DOM element that was created.
///
/// This callback is called before [element] is attached to the DOM, so it can
/// be modified as needed by the Flutter web application.
///
/// See [HtmlElementView.fromTagName] that receives a callback of this type.
///
/// {@template flutter.widgets.web.JSInterop.object}
/// Flutter uses type `Object` so this API doesn't force any JS interop API
/// implementation to Flutter users. This `element` can be cast to any compatible
/// JS interop type as needed. For example: `JSAny` (from `dart:js_interop`),
/// `HTMLElement` (from `package:web`) or any custom JS interop definition.
/// See "Next-generation JS interop": https://dart.dev/interop/js-interop
/// {@endtemplate}
typedef ElementCreatedCallback = void Function(Object element);

/// Embeds an HTML element in the Widget hierarchy in Flutter web.
///
/// The embedded HTML is laid out like any other Flutter widget and
/// transformations (like opacity, and clipping) apply to it as well.
///
/// {@macro flutter.widgets.AndroidView.layout}
///
/// Embedding HTML is a _potentially expensive_ operation and should be avoided
/// when a Flutter equivalent is possible. (See **`isVisible` parameter** below.)
/// This widget is useful to integrate native HTML elements to a Flutter web app,
/// like a `<video>` tag, or a `<div>` where a [Google Map](https://pub.dev/packages/google_maps_flutter)
/// can be rendered.
///
/// This widget **only works on Flutter web.** To embed web content on other
/// platforms, consider using the [`webview_flutter` plugin](https://pub.dev/packages/webview_flutter).
///
/// ## Usage
///
/// There's two ways to use the `HtmlElementView` widget:
///
/// ### `HtmlElementView.fromTagName`
///
/// The [HtmlElementView.fromTagName] constructor creates the HTML element
/// specified by `tagName`, and passes it to the `onElementCreated` callback
/// where it can be customized:
///
/// ```dart
/// // In a `build` method...
/// HtmlElementView.fromTagName(
///   tagName: 'div',
///   onElementCreated: myOnElementCreated,
/// );
/// ```
///
/// The example creates a `<div>` element, then calls the `onElementCreated`
/// callback with the created `<div>`, so it can be customized **before** it is
/// attached to the DOM.
///
/// (See more details about `onElementCreated` in the **Lifecycle** section below.)
///
/// ### Using the `PlatformViewRegistry`
///
/// The primitives used to implement [HtmlElementView.fromTagName] are available
/// for general use through `dart:ui_web`'s `platformViewRegistry`.
///
/// Creating an `HtmlElementView` through these primitives is a two step process:
///
/// #### 1. `registerViewFactory`
///
/// First, a `viewFactory` function needs to be registered for a given `viewType`.
/// Flutter web will call this factory function to create the `element` that will
/// be attached later:
///
/// ```dart
/// import 'dart:ui_web' as ui_web;
/// import 'package:web/web.dart' as web;
///
/// void registerRedDivFactory() {
///   ui_web.platformViewRegistry.registerViewFactory(
///     'my-view-type',
///     (int viewId, {Object? params}) {
///       // Create and return an HTML Element from here
///       final web.HTMLDivElement myDiv = web.HTMLDivElement()
///         ..id = 'some_id_$viewId'
///         ..style.backgroundColor = 'red'
///         ..style.width = '100%'
///         ..style.height = '100%';
///       return myDiv;
///     },
///   );
/// }
/// ```
///
/// `registerViewFactory` **must** be called outside of `build` methods, so the
/// registered function is available when `build` happens.
///
/// See the different types of functions that can be used as `viewFactory`:
///
/// * [`typedef ui_web.PlatformViewFactory`](https://api.flutter.dev/flutter/dart-ui_web/PlatformViewFactory.html)
/// * [`typedef ui_web.ParameterizedPlatformViewFactory`](https://api.flutter.dev/flutter/dart-ui_web/ParameterizedPlatformViewFactory.html)
///
/// #### 2. `HtmlElementView` widget
///
/// Once a factory is registered, an `HtmlElementView` widget of `viewType` can
/// be added to the widget tree, like so:
///
/// ```dart
/// // In a `build` method...
/// const HtmlElementView(
///   viewType: 'my-view-type',
///   onPlatformViewCreated: myOnPlatformViewCreated,
///   creationParams: <String, Object?>{
///     'key': 'someValue',
///   },
/// );
/// ```
///
/// [viewType] **must** match the value used to `registerViewFactory` before.
///
/// [creationParams] (optional) will be passed to your `viewFactory` function,
/// if it accepts them.
///
/// [onPlatformViewCreated] will be called with the `viewId` of the platform
/// view (`element`) created by the `viewFactory`, before it gets attached to
/// the DOM.
///
/// The `viewId` can be used to retrieve the created `element` (The same one
/// passed to `onElementCreated` in [HtmlElementView.fromTagName]) with the
/// `ui_web.platformViewRegistry.`[`getViewById` method](https://api.flutter.dev/flutter/dart-ui_web/PlatformViewRegistry/getViewById.html).
///
/// (See more details about `onPlatformViewCreated` in the **Lifecycle** section
/// below.)
///
/// ## Lifecycle
///
/// `HtmlElementView` widgets behave like any other Flutter stateless widget, but
/// with an additional lifecycle method: `onPlatformViewCreated` / `onElementCreated`
/// (depending on the constructor, see **Usage** above).
///
/// The difference between the two callbacks is the parameter they receive:
///
/// * `onPlatformViewCreated` will be called with the created `viewId` as a parameter,
///   and needs `ui_web.platformViewRegistry.getViewById` to retrieve the created
///   element (See [PlatformViewCreatedCallback]).
/// * `onElementCreated` will be called with the created `element` directly,
///   skipping its `viewId` (See [ElementCreatedCallback]).
///
/// Both callbacks are called **after** the HTML `element` has been created, but
/// **before** it's attached to the DOM.
///
/// ### HTML Lifecycle
///
/// The Browser DOM APIs have additional HTML lifecycle callbacks for the root
/// `element` of an `HtmlElementView`.
///
/// #### Element Attached To The DOM
///
/// It is common for JS code to locate the DOM elements they need with a
/// selector, rather than accepting said DOM elements directly. In those cases,
/// the `element` **must** be attached to the DOM for the selector to work.
///
/// The example below demonstrates **how to create an `onElementAttached` function**
/// that gets called when the root `element` is attached to the DOM using a
/// `ResizeObserver` through `package:web` from the `onElementCreated` lifecycle
/// method:
///
/// ```dart
/// import 'dart:js_interop';
/// import 'package:web/web.dart' as web;
///
/// // Called after `element` is attached to the DOM.
/// void onElementAttached(web.HTMLDivElement element) {
///   final web.Element? located = web.document.querySelector('#someIdThatICanFindLater');
///   assert(located == element, 'Wrong `element` located!');
///   // Do things with `element` or `located`, or call your code now...
///   element.style.backgroundColor = 'green';
/// }
///
/// void onElementCreated(Object element) {
///   element as web.HTMLDivElement;
///   element.style.backgroundColor = 'red';
///   element.id = 'someIdThatICanFindLater';
///
///   // Create the observer
///   final web.ResizeObserver observer = web.ResizeObserver((
///     JSArray<web.ResizeObserverEntry> entries,
///     web.ResizeObserver observer,
///   ) {
///     if (element.isConnected) {
///       // The observer is done, disconnect it.
///       observer.disconnect();
///       // Call our callback.
///       onElementAttached(element);
///     }
///   }.toJS);
///
///   // Connect the observer.
///   observer.observe(element);
/// }
/// ```
///
/// * Read more about [`ResizeObserver` in the MDN](https://developer.mozilla.org/en-US/docs/Web/API/Resize_Observer_API).
///
/// #### Other Observers
///
/// The example above uses a `ResizeObserver` because it can be applied directly
/// to the `element` that is about to be attached. Another observer that could
/// be used for this (with a little bit more code) would be a
/// [`MutationObserver`](https://developer.mozilla.org/en-US/docs/Web/API/MutationObserver).
///
/// The `MutationObserver` requires the parent element in which the `HtmlElementView`
/// is going to be inserted. A safe way to retrieve a parent element for the
/// platform view is to retrieve the `hostElement` of the [FlutterView] where the
/// `HtmlElementView` is being rendered.
///
/// The `hostElement` of the current [FlutterView] can be retrieved through:
///
/// ```dart
/// import 'dart:js_interop';
/// import 'dart:ui_web' as ui_web;
/// import 'package:flutter/widgets.dart';
///
/// void useHostElement(BuildContext context) {
///   final int flutterViewId = View.of(context).viewId;
///   final JSAny? hostElement = ui_web.views.getHostElement(flutterViewId);
///   // Use `package:web` with `hostElement`...
/// }
/// ```
///
/// **Important:** `FlutterView.viewId` and the `viewId` parameter passed to
/// the `viewFactory` identify **different objects**:
///
/// * `flutterViewId` (from `View.of(context)`) represents the [FlutterView]
///   where the web app is currently rendering.
/// * `viewId` (passed to the `viewFactory` function) represents a unique ID
///   for the `HtmlElementView` instance that is being attached to the app.
///
/// Read more about [FlutterView] on Flutter's API docs:
///
/// * [`View.of`](https://api.flutter.dev/flutter/widgets/View/of.html)
/// * [`getHostElement`](https://main-api.flutter.dev/flutter/dart-ui_web/FlutterViewManagerProxy/getHostElement.html)
///
/// ## Pointer events
///
/// In order for the `HtmlElementView` contents to be interactive, they're allowed
/// to handle `pointer-events`. This may result in Flutter missing some events
/// because they've been handled by the `HtmlElementView`, and not seen by
/// Flutter.
///
/// [`package:pointer_interceptor`](https://pub.dev/packages/pointer_interceptor)
/// may help in some cases where Flutter content needs to be overlaid on top of
/// an `HtmlElementView`. Alternatively, the `pointer-events: none` property can
/// be set `onElementCreated`; but that will prevent **ALL** interactions with
/// the underlying HTML content.
///
/// If the `HtmlElementView` is an `<iframe>` element, Flutter will not receive
/// pointer events that land in the `<iframe>` (click/tap, drag, drop, etc.)
/// In those cases, the `HtmlElementView` will seem like it's _swallowing_
/// the events and not participating in Flutter's gesture detection.
///
/// ## `isVisible` parameter
///
/// Rendering custom HTML content (from `HtmlElementView`) in between `canvas`
/// pixels means that the Flutter web engine needs to _split_ the canvas drawing
/// into elements drawn _behind_ the HTML content, and those drawn _above_ it.
///
/// In the Flutter web engine, each of these _splits of the canvas to sandwich
/// HTML content in between_ is referred to as an **overlay**.
///
/// Each _overlay_ present in a scene has implications both in memory and
/// execution performance, and it is best to minimize their amount; browsers
/// support a limited number of _overlays_ on a single scene at a given time.
///
/// `HtmlElementView` objects have an `isVisible` property that can be passed
/// through `registerViewFactory`, or `fromTagName`. `isVisible` refers
/// to whether the `HtmlElementView` will paint pixels on the screen or not.
///
/// Correctly defining this value helps the Flutter web rendering engine optimize
/// the amount of _overlays_ it'll need to render a particular scene.
///
/// In general, `isVisible` should be left to its default value of `true`, but
/// in some `HtmlElementView`s (like the `pointer_interceptor` or `Link` widget),
/// it can be set to `false`, so the engine doesn't _waste_ an overlay to render
/// Flutter content on top of views that don't paint any pixels.
class HtmlElementView extends StatelessWidget {
  /// Creates a platform view for Flutter web.
  ///
  /// `viewType` identifies the type of platform view to create.
  const HtmlElementView({
    super.key,
    required this.viewType,
    this.onPlatformViewCreated,
    this.creationParams,
    this.hitTestBehavior = PlatformViewHitTestBehavior.opaque,
  });

  /// Creates a platform view that creates a DOM element specified by [tagName].
  ///
  /// [isVisible] indicates whether the view is visible to the user or not.
  /// Setting this to false allows the rendering pipeline to perform extra
  /// optimizations knowing that the view will not result in any pixels painted
  /// on the screen.
  ///
  /// [onElementCreated] is called when the DOM element is created. It can be
  /// used by the app to customize the element by adding attributes and styles.
  /// This method is called *before* the element is attached to the DOM.
  factory HtmlElementView.fromTagName({
    Key? key,
    required String tagName,
    bool isVisible = true,
    ElementCreatedCallback? onElementCreated,
    PlatformViewHitTestBehavior hitTestBehavior = PlatformViewHitTestBehavior.opaque,
  }) => HtmlElementViewImpl.createFromTagName(
    key: key,
    tagName: tagName,
    isVisible: isVisible,
    onElementCreated: onElementCreated,
    hitTestBehavior: hitTestBehavior,
  );

  /// The unique identifier for the HTML view type to be embedded by this widget.
  ///
  /// A PlatformViewFactory for this type must have been registered.
  final String viewType;

  /// Callback to invoke after the platform view has been created.
  ///
  /// This method is called *before* the platform view is attached to the DOM.
  ///
  /// May be null.
  final PlatformViewCreatedCallback? onPlatformViewCreated;

  /// Passed as the 2nd argument (i.e. `params`) of the registered view factory.
  final Object? creationParams;

  /// {@macro flutter.widgets.AndroidView.hitTestBehavior}
  final PlatformViewHitTestBehavior hitTestBehavior;

  @override
  Widget build(BuildContext context) => buildImpl(context);
}

class _AndroidViewState extends State<AndroidView> {
  int? _id;
  late AndroidViewController _controller;
  TextDirection? _layoutDirection;
  bool _initialized = false;
  FocusNode? _focusNode;

  static final Set<Factory<OneSequenceGestureRecognizer>> _emptyRecognizersSet =
      <Factory<OneSequenceGestureRecognizer>>{};

  @override
  Widget build(BuildContext context) {
    return Focus(
      focusNode: _focusNode,
      onFocusChange: _onFocusChange,
      child: _AndroidPlatformView(
        controller: _controller,
        hitTestBehavior: widget.hitTestBehavior,
        gestureRecognizers: widget.gestureRecognizers ?? _emptyRecognizersSet,
        clipBehavior: widget.clipBehavior,
      ),
    );
  }

  void _initializeOnce() {
    if (_initialized) {
      return;
    }
    _initialized = true;
    _createNewAndroidView();
    _focusNode = FocusNode(debugLabel: 'AndroidView(id: $_id)');
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final TextDirection newLayoutDirection = _findLayoutDirection();
    final bool didChangeLayoutDirection = _layoutDirection != newLayoutDirection;
    _layoutDirection = newLayoutDirection;

    _initializeOnce();
    if (didChangeLayoutDirection) {
      // The native view will update asynchronously, in the meantime we don't want
      // to block the framework. (so this is intentionally not awaiting).
      _controller.setLayoutDirection(_layoutDirection!);
    }
  }

  @override
  void didUpdateWidget(AndroidView oldWidget) {
    super.didUpdateWidget(oldWidget);

    final TextDirection newLayoutDirection = _findLayoutDirection();
    final bool didChangeLayoutDirection = _layoutDirection != newLayoutDirection;
    _layoutDirection = newLayoutDirection;

    if (widget.viewType != oldWidget.viewType) {
      _controller.disposePostFrame();
      _createNewAndroidView();
      return;
    }

    if (didChangeLayoutDirection) {
      _controller.setLayoutDirection(_layoutDirection!);
    }
  }

  TextDirection _findLayoutDirection() {
    assert(widget.layoutDirection != null || debugCheckHasDirectionality(context));
    return widget.layoutDirection ?? Directionality.of(context);
  }

  @override
  void dispose() {
    _controller.dispose();
    _focusNode?.dispose();
    _focusNode = null;
    super.dispose();
  }

  void _createNewAndroidView() {
    _id = platformViewsRegistry.getNextPlatformViewId();
    _controller = PlatformViewsService.initAndroidView(
      id: _id!,
      viewType: widget.viewType,
      layoutDirection: _layoutDirection!,
      creationParams: widget.creationParams,
      creationParamsCodec: widget.creationParamsCodec,
      onFocus: () {
        _focusNode!.requestFocus();
      },
    );
    if (widget.onPlatformViewCreated != null) {
      _controller.addOnPlatformViewCreatedListener(widget.onPlatformViewCreated!);
    }
  }

  void _onFocusChange(bool isFocused) {
    if (!_controller.isCreated) {
      return;
    }
    if (!isFocused) {
      _controller.clearFocus().catchError((dynamic e) {
        if (e is MissingPluginException) {
          // We land the framework part of Android platform views keyboard
          // support before the engine part. There will be a commit range where
          // clearFocus isn't implemented in the engine. When that happens we
          // just swallow the error here. Once the engine part is rolled to the
          // framework I'll remove this.
          // TODO(amirh): remove this once the engine's clearFocus is rolled.
          return;
        }
      });
      return;
    }
    SystemChannels.textInput
        .invokeMethod<void>('TextInput.setPlatformViewClient', <String, dynamic>{
          'platformViewId': _id,
        })
        .catchError((dynamic e) {
          if (e is MissingPluginException) {
            // We land the framework part of Android platform views keyboard
            // support before the engine part. There will be a commit range where
            // setPlatformViewClient isn't implemented in the engine. When that
            // happens we just swallow the error here. Once the engine part is
            // rolled to the framework I'll remove this.
            // TODO(amirh): remove this once the engine's clearFocus is rolled.
            return;
          }
        });
  }
}

abstract class _DarwinViewState<
  PlatformViewT extends _DarwinView,
  ControllerT extends DarwinPlatformViewController,
  RenderT extends RenderDarwinPlatformView<ControllerT>,
  ViewT extends _DarwinPlatformView<ControllerT, RenderT>
>
    extends State<PlatformViewT> {
  ControllerT? _controller;
  TextDirection? _layoutDirection;
  bool _initialized = false;

  @visibleForTesting
  FocusNode? focusNode;

  static final Set<Factory<OneSequenceGestureRecognizer>> _emptyRecognizersSet =
      <Factory<OneSequenceGestureRecognizer>>{};

  @override
  Widget build(BuildContext context) {
    final ControllerT? controller = _controller;
    if (controller == null) {
      return const SizedBox.expand();
    }
    return Focus(
      focusNode: focusNode,
      onFocusChange: (bool isFocused) => _onFocusChange(isFocused, controller),
      child: childPlatformView(),
    );
  }

  ViewT childPlatformView();

  void _initializeOnce() {
    if (_initialized) {
      return;
    }
    _initialized = true;
    _createNewUiKitView();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final TextDirection newLayoutDirection = _findLayoutDirection();
    final bool didChangeLayoutDirection = _layoutDirection != newLayoutDirection;
    _layoutDirection = newLayoutDirection;

    _initializeOnce();
    if (didChangeLayoutDirection) {
      // The native view will update asynchronously, in the meantime we don't want
      // to block the framework. (so this is intentionally not awaiting).
      _controller?.setLayoutDirection(_layoutDirection!);
    }
  }

  @override
  void didUpdateWidget(PlatformViewT oldWidget) {
    super.didUpdateWidget(oldWidget);

    final TextDirection newLayoutDirection = _findLayoutDirection();
    final bool didChangeLayoutDirection = _layoutDirection != newLayoutDirection;
    _layoutDirection = newLayoutDirection;

    if (widget.viewType != oldWidget.viewType) {
      _controller?.dispose();
      _controller = null;
      focusNode?.dispose();
      focusNode = null;
      _createNewUiKitView();
      return;
    }

    if (didChangeLayoutDirection) {
      _controller?.setLayoutDirection(_layoutDirection!);
    }
  }

  TextDirection _findLayoutDirection() {
    assert(widget.layoutDirection != null || debugCheckHasDirectionality(context));
    return widget.layoutDirection ?? Directionality.of(context);
  }

  @override
  void dispose() {
    _controller?.dispose();
    _controller = null;
    focusNode?.dispose();
    focusNode = null;
    super.dispose();
  }

  Future<void> _createNewUiKitView() async {
    final int id = platformViewsRegistry.getNextPlatformViewId();
    final ControllerT controller = await createNewViewController(id);
    if (!mounted) {
      controller.dispose();
      return;
    }
    widget.onPlatformViewCreated?.call(id);
    setState(() {
      _controller = controller;
      focusNode = FocusNode(debugLabel: 'UiKitView(id: $id)');
    });
  }

  Future<ControllerT> createNewViewController(int id);

  void _onFocusChange(bool isFocused, ControllerT controller) {
    if (!isFocused) {
      // Unlike Android, we do not need to send "clearFocus" channel message
      // to the engine, because focusing on another view will automatically
      // cancel the focus on the previously focused platform view.
      return;
    }
    SystemChannels.textInput.invokeMethod<void>(
      'TextInput.setPlatformViewClient',
      <String, dynamic>{'platformViewId': controller.id},
    );
  }
}

class _UiKitViewState
    extends _DarwinViewState<UiKitView, UiKitViewController, RenderUiKitView, _UiKitPlatformView> {
  @override
  Future<UiKitViewController> createNewViewController(int id) async {
    return PlatformViewsService.initUiKitView(
      id: id,
      viewType: widget.viewType,
      layoutDirection: _layoutDirection!,
      creationParams: widget.creationParams,
      creationParamsCodec: widget.creationParamsCodec,
      onFocus: () {
        focusNode?.requestFocus();
      },
    );
  }

  @override
  _UiKitPlatformView childPlatformView() {
    return _UiKitPlatformView(
      controller: _controller!,
      hitTestBehavior: widget.hitTestBehavior,
      gestureRecognizers: widget.gestureRecognizers ?? _DarwinViewState._emptyRecognizersSet,
    );
  }
}

class _AppKitViewState
    extends
        _DarwinViewState<AppKitView, AppKitViewController, RenderAppKitView, _AppKitPlatformView> {
  @override
  Future<AppKitViewController> createNewViewController(int id) async {
    return PlatformViewsService.initAppKitView(
      id: id,
      viewType: widget.viewType,
      layoutDirection: _layoutDirection!,
      creationParams: widget.creationParams,
      creationParamsCodec: widget.creationParamsCodec,
      onFocus: () {
        focusNode?.requestFocus();
      },
    );
  }

  @override
  _AppKitPlatformView childPlatformView() {
    return _AppKitPlatformView(
      controller: _controller!,
      hitTestBehavior: widget.hitTestBehavior,
      gestureRecognizers: widget.gestureRecognizers ?? _DarwinViewState._emptyRecognizersSet,
    );
  }
}

class _AndroidPlatformView extends LeafRenderObjectWidget {
  const _AndroidPlatformView({
    required this.controller,
    required this.hitTestBehavior,
    required this.gestureRecognizers,
    this.clipBehavior = Clip.hardEdge,
  });

  final AndroidViewController controller;
  final PlatformViewHitTestBehavior hitTestBehavior;
  final Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers;
  final Clip clipBehavior;

  @override
  RenderObject createRenderObject(BuildContext context) => RenderAndroidView(
    viewController: controller,
    hitTestBehavior: hitTestBehavior,
    gestureRecognizers: gestureRecognizers,
    clipBehavior: clipBehavior,
  );

  @override
  void updateRenderObject(BuildContext context, RenderAndroidView renderObject) {
    renderObject.controller = controller;
    renderObject.hitTestBehavior = hitTestBehavior;
    renderObject.updateGestureRecognizers(gestureRecognizers);
    renderObject.clipBehavior = clipBehavior;
  }
}

abstract class _DarwinPlatformView<
  TController extends DarwinPlatformViewController,
  TRender extends RenderDarwinPlatformView<TController>
>
    extends LeafRenderObjectWidget {
  const _DarwinPlatformView({
    required this.controller,
    required this.hitTestBehavior,
    required this.gestureRecognizers,
  });

  final TController controller;
  final PlatformViewHitTestBehavior hitTestBehavior;
  final Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers;

  @override
  @mustCallSuper
  void updateRenderObject(BuildContext context, TRender renderObject) {
    renderObject
      ..viewController = controller
      ..hitTestBehavior = hitTestBehavior
      ..updateGestureRecognizers(gestureRecognizers);
  }
}

class _UiKitPlatformView extends _DarwinPlatformView<UiKitViewController, RenderUiKitView> {
  const _UiKitPlatformView({
    required super.controller,
    required super.hitTestBehavior,
    required super.gestureRecognizers,
  });

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderUiKitView(
      viewController: controller,
      hitTestBehavior: hitTestBehavior,
      gestureRecognizers: gestureRecognizers,
    );
  }
}

class _AppKitPlatformView extends _DarwinPlatformView<AppKitViewController, RenderAppKitView> {
  const _AppKitPlatformView({
    required super.controller,
    required super.hitTestBehavior,
    required super.gestureRecognizers,
  });

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderAppKitView(
      viewController: controller,
      hitTestBehavior: hitTestBehavior,
      gestureRecognizers: gestureRecognizers,
    );
  }
}

/// The parameters used to create a [PlatformViewController].
///
/// See also:
///
///  * [CreatePlatformViewCallback] which uses this object to create a [PlatformViewController].
class PlatformViewCreationParams {
  const PlatformViewCreationParams._({
    required this.id,
    required this.viewType,
    required this.onPlatformViewCreated,
    required this.onFocusChanged,
  });

  /// The unique identifier for the new platform view.
  ///
  /// [PlatformViewController.viewId] should match this id.
  final int id;

  /// The unique identifier for the type of platform view to be embedded.
  ///
  /// This viewType is used to tell the platform which type of view to
  /// associate with the [id].
  final String viewType;

  /// Callback invoked after the platform view has been created.
  final PlatformViewCreatedCallback onPlatformViewCreated;

  /// Callback invoked when the platform view's focus is changed on the platform side.
  ///
  /// The value is true when the platform view gains focus and false when it loses focus.
  final ValueChanged<bool> onFocusChanged;
}

/// A factory for a surface presenting a platform view as part of the widget hierarchy.
///
/// The returned widget should present the platform view associated with `controller`.
///
/// See also:
///
///  * [PlatformViewSurface], a common widget for presenting platform views.
typedef PlatformViewSurfaceFactory =
    Widget Function(BuildContext context, PlatformViewController controller);

/// Constructs a [PlatformViewController].
///
/// The [PlatformViewController.viewId] field of the created controller must match the value of the
/// params [PlatformViewCreationParams.id] field.
///
/// See also:
///
///  * [PlatformViewLink], which links a platform view with the Flutter framework.
typedef CreatePlatformViewCallback =
    PlatformViewController Function(PlatformViewCreationParams params);

/// Links a platform view with the Flutter framework.
///
/// Provides common functionality for embedding a platform view (e.g an android.view.View on Android)
/// with the Flutter framework.
///
/// {@macro flutter.widgets.AndroidView.lifetime}
///
/// To implement a new platform view widget, return this widget in the `build` method.
/// For example:
///
/// ```dart
/// class FooPlatformView extends StatelessWidget {
///   const FooPlatformView({super.key});
///   @override
///   Widget build(BuildContext context) {
///     return PlatformViewLink(
///       viewType: 'webview',
///       onCreatePlatformView: createFooWebView,
///       surfaceFactory: (BuildContext context, PlatformViewController controller) {
///         return PlatformViewSurface(
///           gestureRecognizers: gestureRecognizers,
///           controller: controller,
///           hitTestBehavior: PlatformViewHitTestBehavior.opaque,
///         );
///       },
///    );
///   }
/// }
/// ```
///
/// The `surfaceFactory` and the `onCreatePlatformView` are only called when the
/// state of this widget is initialized, or when the `viewType` changes.
class PlatformViewLink extends StatefulWidget {
  /// Construct a [PlatformViewLink] widget.
  ///
  /// See also:
  ///
  ///  * [PlatformViewSurface] for details on the widget returned by `surfaceFactory`.
  ///  * [PlatformViewCreationParams] for how each parameter can be used when implementing `createPlatformView`.
  const PlatformViewLink({
    super.key,
    required PlatformViewSurfaceFactory surfaceFactory,
    required CreatePlatformViewCallback onCreatePlatformView,
    required this.viewType,
  }) : _surfaceFactory = surfaceFactory,
       _onCreatePlatformView = onCreatePlatformView;

  final PlatformViewSurfaceFactory _surfaceFactory;
  final CreatePlatformViewCallback _onCreatePlatformView;

  /// The unique identifier for the view type to be embedded.
  ///
  /// Typically, this viewType has already been registered on the platform side.
  final String viewType;

  @override
  State<StatefulWidget> createState() => _PlatformViewLinkState();
}

class _PlatformViewLinkState extends State<PlatformViewLink> {
  int? _id;
  PlatformViewController? _controller;
  bool _platformViewCreated = false;
  Widget? _surface;
  FocusNode? _focusNode;

  @override
  Widget build(BuildContext context) {
    final PlatformViewController? controller = _controller;
    if (controller == null) {
      return const SizedBox.expand();
    }
    if (!_platformViewCreated) {
      // Depending on the implementation, the first non-empty size can be used
      // to size the platform view.
      return _PlatformViewPlaceHolder(
        onLayout: (Size size, Offset position) {
          if (controller.awaitingCreation && !size.isEmpty) {
            controller.create(size: size, position: position);
          }
        },
      );
    }
    _surface ??= widget._surfaceFactory(context, controller);
    return Focus(
      focusNode: _focusNode,
      onFocusChange: _handleFrameworkFocusChanged,
      child: _surface!,
    );
  }

  @override
  void initState() {
    _focusNode = FocusNode(debugLabel: 'PlatformView(id: $_id)');
    _initialize();
    super.initState();
  }

  @override
  void didUpdateWidget(PlatformViewLink oldWidget) {
    super.didUpdateWidget(oldWidget);

    if (widget.viewType != oldWidget.viewType) {
      _controller?.disposePostFrame();
      // The _surface has to be recreated as its controller is disposed.
      // Setting _surface to null will trigger its creation in build().
      _surface = null;
      _initialize();
    }
  }

  void _initialize() {
    _id = platformViewsRegistry.getNextPlatformViewId();
    _controller = widget._onCreatePlatformView(
      PlatformViewCreationParams._(
        id: _id!,
        viewType: widget.viewType,
        onPlatformViewCreated: _onPlatformViewCreated,
        onFocusChanged: _handlePlatformFocusChanged,
      ),
    );
  }

  void _onPlatformViewCreated(int id) {
    if (mounted) {
      setState(() {
        _platformViewCreated = true;
      });
    }
  }

  void _handleFrameworkFocusChanged(bool isFocused) {
    if (!isFocused) {
      _controller?.clearFocus();
    }
    SystemChannels.textInput.invokeMethod<void>(
      'TextInput.setPlatformViewClient',
      <String, dynamic>{'platformViewId': _id},
    );
  }

  void _handlePlatformFocusChanged(bool isFocused) {
    if (isFocused) {
      _focusNode!.requestFocus();
    }
  }

  @override
  void dispose() {
    _controller?.dispose();
    _controller = null;
    _focusNode?.dispose();
    _focusNode = null;
    super.dispose();
  }
}

/// Integrates a platform view with Flutter's compositor, touch, and semantics subsystems.
///
/// The compositor integration is done by adding a [PlatformViewLayer] to the layer tree. [PlatformViewSurface]
/// isn't supported on all platforms (e.g on Android platform views can be composited by using a [TextureLayer] or
/// [AndroidViewSurface]).
/// Custom Flutter embedders can support [PlatformViewLayer]s by implementing a SystemCompositor.
///
/// The widget fills all available space, the parent of this object must provide bounded layout
/// constraints.
///
/// If the associated platform view is not created the [PlatformViewSurface] does not paint any contents.
///
/// See also:
///
///  * [AndroidView] which embeds an Android platform view in the widget hierarchy using a [TextureLayer].
///  * [UiKitView] which embeds an iOS platform view in the widget hierarchy.
// TODO(amirh): Link to the embedder's system compositor documentation once available.
class PlatformViewSurface extends LeafRenderObjectWidget {
  /// Construct a [PlatformViewSurface].
  const PlatformViewSurface({
    super.key,
    required this.controller,
    required this.hitTestBehavior,
    required this.gestureRecognizers,
  });

  /// The controller for the platform view integrated by this [PlatformViewSurface].
  ///
  /// [PlatformViewController] is used for dispatching touch events to the platform view.
  /// [PlatformViewController.viewId] identifies the platform view whose contents are painted by this widget.
  final PlatformViewController controller;

  /// Which gestures should be forwarded to the PlatformView.
  ///
  /// {@macro flutter.widgets.AndroidView.gestureRecognizers.descHead}
  ///
  /// For example, with the following setup vertical drags will not be dispatched to the platform view
  /// as the vertical drag gesture is claimed by the parent [GestureDetector].
  ///
  /// ```dart
  /// GestureDetector(
  ///   onVerticalDragStart: (DragStartDetails details) { },
  ///   child: PlatformViewSurface(
  ///     gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
  ///     controller: _controller,
  ///     hitTestBehavior: PlatformViewHitTestBehavior.opaque,
  ///   ),
  /// )
  /// ```
  ///
  /// To get the [PlatformViewSurface] to claim the vertical drag gestures we can pass a vertical drag
  /// gesture recognizer factory in [gestureRecognizers] e.g:
  ///
  /// ```dart
  /// GestureDetector(
  ///   onVerticalDragStart: (DragStartDetails details) { },
  ///   child: SizedBox(
  ///     width: 200.0,
  ///     height: 100.0,
  ///     child: PlatformViewSurface(
  ///       gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
  ///         Factory<OneSequenceGestureRecognizer>(
  ///           () => EagerGestureRecognizer(),
  ///         ),
  ///       },
  ///       controller: _controller,
  ///       hitTestBehavior: PlatformViewHitTestBehavior.opaque,
  ///     ),
  ///   ),
  /// )
  /// ```
  ///
  /// {@macro flutter.widgets.AndroidView.gestureRecognizers.descFoot}
  // We use OneSequenceGestureRecognizers as they support gesture arena teams.
  // TODO(amirh): get a list of GestureRecognizers here.
  // https://github.com/flutter/flutter/issues/20953
  final Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers;

  /// {@macro flutter.widgets.AndroidView.hitTestBehavior}
  final PlatformViewHitTestBehavior hitTestBehavior;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return PlatformViewRenderBox(
      controller: controller,
      gestureRecognizers: gestureRecognizers,
      hitTestBehavior: hitTestBehavior,
    );
  }

  @override
  void updateRenderObject(BuildContext context, PlatformViewRenderBox renderObject) {
    renderObject
      ..controller = controller
      ..hitTestBehavior = hitTestBehavior
      ..updateGestureRecognizers(gestureRecognizers);
  }
}

/// Integrates an Android view with Flutter's compositor, touch, and semantics subsystems.
///
/// The compositor integration is done by adding a [TextureLayer] to the layer tree.
///
/// The parent of this object must provide bounded layout constraints.
///
/// If the associated platform view is not created, the [AndroidViewSurface] does not paint any contents.
///
/// When possible, you may want to use [AndroidView] directly, since it requires less boilerplate code
/// than [AndroidViewSurface], and there's no difference in performance, or other trade-off(s).
///
/// See also:
///
///  * [AndroidView] which embeds an Android platform view in the widget hierarchy.
///  * [UiKitView] which embeds an iOS platform view in the widget hierarchy.
class AndroidViewSurface extends StatefulWidget {
  /// Construct an `AndroidPlatformViewSurface`.
  const AndroidViewSurface({
    super.key,
    required this.controller,
    required this.hitTestBehavior,
    required this.gestureRecognizers,
  });

  /// The controller for the platform view integrated by this [AndroidViewSurface].
  ///
  /// See [PlatformViewSurface.controller] for details.
  final AndroidViewController controller;

  /// Which gestures should be forwarded to the PlatformView.
  ///
  /// See [PlatformViewSurface.gestureRecognizers] for details.
  final Set<Factory<OneSequenceGestureRecognizer>> gestureRecognizers;

  /// {@macro flutter.widgets.AndroidView.hitTestBehavior}
  final PlatformViewHitTestBehavior hitTestBehavior;

  @override
  State<StatefulWidget> createState() {
    return _AndroidViewSurfaceState();
  }
}

class _AndroidViewSurfaceState extends State<AndroidViewSurface> {
  @override
  void initState() {
    super.initState();
    if (!widget.controller.isCreated) {
      // Schedule a rebuild once creation is complete and the final display
      // type is known.
      widget.controller.addOnPlatformViewCreatedListener(_onPlatformViewCreated);
    }
  }

  @override
  void dispose() {
    widget.controller.removeOnPlatformViewCreatedListener(_onPlatformViewCreated);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (widget.controller.requiresViewComposition) {
      return _PlatformLayerBasedAndroidViewSurface(
        controller: widget.controller,
        hitTestBehavior: widget.hitTestBehavior,
        gestureRecognizers: widget.gestureRecognizers,
      );
    } else {
      return _TextureBasedAndroidViewSurface(
        controller: widget.controller,
        hitTestBehavior: widget.hitTestBehavior,
        gestureRecognizers: widget.gestureRecognizers,
      );
    }
  }

  void _onPlatformViewCreated(int _) {
    // Trigger a re-build based on the current controller state.
    setState(() {});
  }
}

// Displays an Android platform view via GL texture.
class _TextureBasedAndroidViewSurface extends PlatformViewSurface {
  const _TextureBasedAndroidViewSurface({
    required AndroidViewController super.controller,
    required super.hitTestBehavior,
    required super.gestureRecognizers,
  });

  @override
  RenderObject createRenderObject(BuildContext context) {
    final AndroidViewController viewController = controller as AndroidViewController;
    // Use GL texture based composition.
    // App should use GL texture unless they require to embed a SurfaceView.
    final RenderAndroidView renderBox = RenderAndroidView(
      viewController: viewController,
      gestureRecognizers: gestureRecognizers,
      hitTestBehavior: hitTestBehavior,
    );
    viewController.pointTransformer = (Offset position) => renderBox.globalToLocal(position);
    return renderBox;
  }
}

class _PlatformLayerBasedAndroidViewSurface extends PlatformViewSurface {
  const _PlatformLayerBasedAndroidViewSurface({
    required AndroidViewController super.controller,
    required super.hitTestBehavior,
    required super.gestureRecognizers,
  });

  @override
  RenderObject createRenderObject(BuildContext context) {
    final AndroidViewController viewController = controller as AndroidViewController;
    final PlatformViewRenderBox renderBox =
        super.createRenderObject(context) as PlatformViewRenderBox;
    viewController.pointTransformer = (Offset position) => renderBox.globalToLocal(position);
    return renderBox;
  }
}

/// A callback used to notify the size of the platform view placeholder.
/// This size is the initial size of the platform view.
typedef _OnLayoutCallback = void Function(Size size, Offset position);

/// A [RenderBox] that notifies its size to the owner after a layout.
class _PlatformViewPlaceholderBox extends RenderConstrainedBox {
  _PlatformViewPlaceholderBox({required this.onLayout})
    : super(
        additionalConstraints: const BoxConstraints.tightFor(
          width: double.infinity,
          height: double.infinity,
        ),
      );

  _OnLayoutCallback onLayout;

  @override
  void performLayout() {
    super.performLayout();
    // A call to `localToGlobal` requires waiting for a frame to render first.
    SchedulerBinding.instance.addPostFrameCallback((_) {
      onLayout(size, localToGlobal(Offset.zero));
    }, debugLabel: 'PlatformViewPlaceholderBox.onLayout');
  }
}

/// When a platform view is in the widget hierarchy, this widget is used to capture
/// the size of the platform view after the first layout.
/// This placeholder is basically a [SizedBox.expand] with a [onLayout] callback to
/// notify the size of the render object to its parent.
class _PlatformViewPlaceHolder extends SingleChildRenderObjectWidget {
  const _PlatformViewPlaceHolder({required this.onLayout});

  final _OnLayoutCallback onLayout;

  @override
  _PlatformViewPlaceholderBox createRenderObject(BuildContext context) {
    return _PlatformViewPlaceholderBox(onLayout: onLayout);
  }

  @override
  void updateRenderObject(BuildContext context, _PlatformViewPlaceholderBox renderObject) {
    renderObject.onLayout = onLayout;
  }
}

extension on PlatformViewController {
  /// Disposes the controller in a post-frame callback, to allow other widgets to
  /// remove their listeners before the controller is disposed.
  void disposePostFrame() {
    SchedulerBinding.instance.addPostFrameCallback((_) {
      dispose();
    }, debugLabel: 'PlatformViewController.dispose');
  }
}
