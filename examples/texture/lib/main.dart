// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

class TexturePage extends StatefulWidget {
  const TexturePage({super.key});

  @override
  State<TexturePage> createState() => _TexturePageState();
}

class _TexturePageState extends State<TexturePage> {
  static const int textureWidth = 300;
  static const int textureHeight = 300;
  static const MethodChannel channel = MethodChannel('samples.flutter.io/texture');
  final Future<int?> textureId = channel.invokeMethod('create', <int>[textureWidth, textureHeight]);

  // Set the color of the texture.
  Future<void> setColor(int r, int g, int b) async {
    await channel.invokeMethod('setColor', <int>[r, g, b]);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: const Text('Texture Example'),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          spacing: 10,
          children: <Widget>[
            FutureBuilder<int?>(
              future: textureId,
              builder: (BuildContext context, AsyncSnapshot<int?> snapshot) {
                if (!snapshot.hasData) {
                  return const Text('Creating texture...');
                }

                if (snapshot.data == null) {
                  return const Text('Error creating texture');
                }

                return SizedBox(
                  width: textureWidth.toDouble(),
                  height: textureHeight.toDouble(),
                  child: Texture(textureId: snapshot.data!),
                );
              },
            ),
            OutlinedButton(
              child: const Text('Flutter Navy'),
              onPressed: () => setColor(0x04, 0x2b, 0x59),
            ),
            OutlinedButton(
              child: const Text('Flutter Blue'),
              onPressed: () => setColor(0x05, 0x53, 0xb1),
            ),
            OutlinedButton(
              child: const Text('Flutter Sky'),
              onPressed: () => setColor(0x02, 0x7d, 0xfd),
            ),
            OutlinedButton(child: const Text('Red'), onPressed: () => setColor(0xf2, 0x5d, 0x50)),
            OutlinedButton(
              child: const Text('Yellow'),
              onPressed: () => setColor(0xff, 0xf2, 0x75),
            ),
            OutlinedButton(
              child: const Text('Purple'),
              onPressed: () => setColor(0x62, 0x00, 0xee),
            ),
            OutlinedButton(child: const Text('Green'), onPressed: () => setColor(0x1c, 0xda, 0xc5)),
          ],
        ),
      ),
    );
  }
}

void main() {
  runApp(const MaterialApp(home: TexturePage()));
}
