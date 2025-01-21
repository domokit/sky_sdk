// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:android_driver_extensions/extension.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_driver/driver_extension.dart';

import '../src/allow_list_devices.dart';

const MethodChannel _channel = MethodChannel('smiley_face_texture');
Future<int> _fetchTexture(int width, int height) async {
  final int? result = await _channel.invokeMethod<int>('initTexture', <String, int>{
    'width': width,
    'height': height,
  });
  return result!;
}

void main() async {
  ensureAndroidDevice();
  enableFlutterDriverExtension(commands: <CommandExtension>[nativeDriverCommands]);

  // Run on full screen.
  SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersive);

  // Fetch the texture ID.
  final Future<int> textureId = _fetchTexture(512, 512);
  runApp(MainApp(textureId));
}

final class MainApp extends StatelessWidget {
  const MainApp(this.textureId, {super.key});
  final Future<int> textureId;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: FutureBuilder<int>(
        future: textureId,
        builder: (BuildContext context, AsyncSnapshot<int> snapshot) {
          if (snapshot.hasData) {
            return Texture(textureId: snapshot.data!);
          }
          return const CircularProgressIndicator();
        },
      ),
    );
  }
}
