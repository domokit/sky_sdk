// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import '../dom.dart';
import '../platform_dispatcher.dart';
import '../services.dart';
import '../util.dart';
import 'content_manager.dart';
import 'slots.dart';

/// The signature for a callback for a Platform Message. From the `ui` package.
/// Copied here so there's no circular dependencies.
typedef _PlatformMessageResponseCallback = void Function(ByteData? data);

/// A function that handle a newly created [DomElement] with the contents of a
/// platform view with a unique [int] id.
typedef PlatformViewContentHandler = void Function(DomElement);

/// This class handles incoming framework messages to create/dispose Platform Views.
///
/// (The instance of this class is connected to the `flutter/platform_views`
/// Platform Channel in the [EnginePlatformDispatcher] class.)
///
/// It uses a [PlatformViewManager] to handle the CRUD of the DOM of Platform Views.
/// This `contentManager` is shared across the engine, to perform
/// all operations related to platform views (registration, rendering, etc...),
/// regardless of the rendering backend.
///
/// Platform views are injected into the DOM when needed by the correct instance
/// of the active renderer.
///
/// The rendering and compositing of Platform Views can create the other "half" of a
/// Platform View: the `slot`, through the [createPlatformViewSlot] method.
///
/// When a Platform View is disposed of, it is removed from the cache (and DOM)
/// directly by the `contentManager`. The canvaskit rendering backend needs to do
/// some extra cleanup of its internal state, but it can do it automatically. See
/// [HtmlViewEmbedder.disposeViews].
class PlatformViewMessageHandler {
  PlatformViewMessageHandler({required PlatformViewManager contentManager})
    : _contentManager = contentManager;

  static const String channelName = 'flutter/platform_views';

  /// The shared instance of PlatformViewMessageHandler.
  ///
  /// Unless configured differently, this connects to the shared instance of the
  /// [PlatformViewManager].
  static PlatformViewMessageHandler instance = PlatformViewMessageHandler(
    contentManager: PlatformViewManager.instance,
  );

  final MethodCodec _codec = const StandardMethodCodec();
  final PlatformViewManager _contentManager;

  /// Handle a `create` Platform View message.
  ///
  /// This will attempt to render the `contents` of a Platform View, if its
  /// `viewType` has been registered previously.
  ///
  /// (See [PlatformViewManager.registerFactory] for more details.)
  ///
  /// If all goes well, this function will `callback` with an empty success envelope.
  /// In case of error, this will `callback` with an error envelope describing the error.
  ///
  /// The `callback` signals when the contents of a given [platformViewId] have
  /// been rendered. They're now accessible through `platformViewRegistry.getViewById`
  /// from `dart:ui_web`. **(Not the DOM!)**
  void _createPlatformView(
    _PlatformMessageResponseCallback callback, {
    required int platformViewId,
    required String platformViewType,
    required Object? params,
  }) {
    if (!_contentManager.knowsViewType(platformViewType)) {
      callback(
        _codec.encodeErrorEnvelope(
          code: 'unregistered_view_type',
          message:
              'A HtmlElementView widget is trying to create a platform view '
              'with an unregistered type: <$platformViewType>.',
          details:
              'If you are the author of the PlatformView, make sure '
              '`registerViewFactory` is invoked.',
        ),
      );
      return;
    }

    if (_contentManager.knowsViewId(platformViewId)) {
      callback(
        _codec.encodeErrorEnvelope(
          code: 'recreating_view',
          message: 'trying to create an already created view',
          details: 'view id: $platformViewId',
        ),
      );
      return;
    }

    _contentManager.renderContent(platformViewType, platformViewId, params);

    callback(_codec.encodeSuccessEnvelope(null));
  }

  /// Handle a `dispose` Platform View message.
  ///
  /// This will clear the cached information that the framework has about a given
  /// `platformViewId`, through the [_contentManager].
  ///
  /// Once that's done, the dispose call is delegated to the [_disposeHandler]
  /// function, so the active rendering backend can dispose of whatever resources
  /// it needed to get ahold of.
  ///
  /// This function should always `callback` with an empty success envelope.
  void _disposePlatformView(
    _PlatformMessageResponseCallback callback, {
    required int platformViewId,
  }) {
    // The contentManager removes the slot and the contents from its internal
    // cache, and the DOM.
    _contentManager.clearPlatformView(platformViewId);

    callback(_codec.encodeSuccessEnvelope(null));
  }

  /// Handles legacy PlatformViewCalls that don't contain a Flutter View ID.
  ///
  /// This is transitional code to support the old platform view channel. As
  /// soon as the framework code is updated to send the Flutter View ID, this
  /// method can be removed.
  void handlePlatformViewCall(
    String method,
    dynamic arguments,
    _PlatformMessageResponseCallback callback,
  ) {
    switch (method) {
      case 'create':
        arguments as Map<dynamic, dynamic>;
        _createPlatformView(
          callback,
          platformViewId: arguments.readInt('id'),
          platformViewType: arguments.readString('viewType'),
          params: arguments['params'],
        );
        return;
      // TODO(web): Send `arguments` as a Map for `dispose` too!
      case 'dispose':
        _disposePlatformView(callback, platformViewId: arguments as int);
        return;
    }
    callback(null);
  }
}
