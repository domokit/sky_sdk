// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:android_driver_extensions/extension.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_driver/driver_extension.dart';

import '../src/allow_list_devices.dart';

void main() async {
  ensureAndroidDevice();
  enableFlutterDriverExtension(
    handler: (String? command) async {
      return json.encode(<String, Object?>{
        'supported': await HybridAndroidViewController.checkIfSupported(),
      });
    },
    commands: <CommandExtension>[nativeDriverCommands],
  );

  // Run on full screen.
  await SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersive);
  runApp(const MainApp());
}

// This should appear as the yellow line over a blue box. The
// green box should not be visible unless the platform view has not loaded yet.
final class MainApp extends StatefulWidget {
  const MainApp({super.key});

  @override
  State<MainApp> createState() => _MainAppState();
}

class _MainAppState extends State<MainApp> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Stack(
        children: <Widget>[
          Positioned.directional(
            top: 100,
            textDirection: TextDirection.ltr,
            child: const SizedBox(
              width: 200,
              height: 200,
              child: _HybridCompositionAndroidPlatformView(viewType: 'box_platform_view'),
            ),
          ),
          Positioned.directional(
            top: 200,
            textDirection: TextDirection.ltr,
            child: const SizedBox(width: 800, height: 200, child: ColoredBox(color: Colors.yellow)),
          ),
          Positioned.directional(
            top: 300,
            textDirection: TextDirection.ltr,
            child: const SizedBox(
              width: 200,
              height: 200,
              child: _HybridCompositionAndroidPlatformView(viewType: 'box_platform_view'),
            ),
          ),
          Positioned.directional(
            top: 400,
            textDirection: TextDirection.ltr,
            child: const SizedBox(width: 800, height: 200, child: ColoredBox(color: Colors.red)),
          ),
          Positioned.directional(
            top: 500,
            textDirection: TextDirection.ltr,
            child: const SizedBox(
              width: 200,
              height: 200,
              child: _HybridCompositionAndroidPlatformView(viewType: 'box_platform_view'),
            ),
          ),
          Positioned.directional(
            top: 600,
            textDirection: TextDirection.ltr,
            child: const SizedBox(width: 800, height: 200, child: ColoredBox(color: Colors.orange)),
          ),
        ],
      ),
    );
  }
}

final class _HybridCompositionAndroidPlatformView extends StatelessWidget {
  const _HybridCompositionAndroidPlatformView({required this.viewType});

  final String viewType;

  @override
  Widget build(BuildContext context) {
    return PlatformViewLink(
      viewType: viewType,
      surfaceFactory: (BuildContext context, PlatformViewController controller) {
        return AndroidViewSurface(
          controller: controller as AndroidViewController,
          gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
          hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        );
      },
      onCreatePlatformView: (PlatformViewCreationParams params) {
        return PlatformViewsService.initHybridAndroidView(
            id: params.id,
            viewType: viewType,
            layoutDirection: TextDirection.ltr,
            creationParamsCodec: const StandardMessageCodec(),
          )
          ..addOnPlatformViewCreatedListener(params.onPlatformViewCreated)
          ..create();
      },
    );
  }
}
