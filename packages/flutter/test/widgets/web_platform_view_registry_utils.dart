// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui_web' as ui_web;

import 'package:collection/collection.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

typedef FakeViewFactory = ({String viewType, bool isVisible, Function viewFactory});

typedef FakePlatformView = ({int id, String viewType, Object? params, Object htmlElement});

class FakePlatformViewRegistry implements ui_web.PlatformViewRegistry {
  FakePlatformViewRegistry() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform_views,
      _onMethodCall,
    );
  }

  Set<FakePlatformView> get views => Set<FakePlatformView>.unmodifiable(_views);
  final Set<FakePlatformView> _views = <FakePlatformView>{};

  final Set<FakeViewFactory> _registeredViewTypes = <FakeViewFactory>{};

  @override
  bool registerViewFactory(String viewType, Function viewFactory, {bool isVisible = true}) {
    if (_findRegisteredViewFactory(viewType) != null) {
      return false;
    }
    _registeredViewTypes.add((viewType: viewType, isVisible: isVisible, viewFactory: viewFactory));
    return true;
  }

  @override
  Object getViewById(int viewId) {
    return _findViewById(viewId)!.htmlElement;
  }

  FakeViewFactory? _findRegisteredViewFactory(String viewType) {
    return _registeredViewTypes.singleWhereOrNull(
      (FakeViewFactory registered) => registered.viewType == viewType,
    );
  }

  FakePlatformView? _findViewById(int viewId) {
    return _views.singleWhereOrNull((FakePlatformView view) => view.id == viewId);
  }

  Future<dynamic> _onMethodCall(MethodCall call) {
    return switch (call.method) {
      'create' => _create(call),
      'dispose' => _dispose(call),
      _ => Future<dynamic>.sync(() => null),
    };
  }

  Future<dynamic> _create(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    final String viewType = args['viewType'] as String;
    final Object? params = args['params'];

    if (_findViewById(id) != null) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to create an already created platform view, view id: $id',
      );
    }

    final FakeViewFactory? registered = _findRegisteredViewFactory(viewType);
    if (registered == null) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to create a platform view of unregistered type: $viewType',
      );
    }

    final ui_web.ParameterizedPlatformViewFactory viewFactory =
        registered.viewFactory as ui_web.ParameterizedPlatformViewFactory;

    _views.add((
      id: id,
      viewType: viewType,
      params: params,
      htmlElement: viewFactory(id, params: params),
    ));
    return null;
  }

  Future<dynamic> _dispose(MethodCall call) async {
    final int id = call.arguments as int;

    final FakePlatformView? view = _findViewById(id);
    if (view == null) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to dispose a platform view with unknown id: $id',
      );
    }

    _views.remove(view);
    return null;
  }
}
