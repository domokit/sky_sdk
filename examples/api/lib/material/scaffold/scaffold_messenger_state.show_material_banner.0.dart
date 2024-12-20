// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ScaffoldMessengerState.showMaterialBanner].

void main() => runApp(const ShowMaterialBannerExampleApp());

class ShowMaterialBannerExampleApp extends StatelessWidget {
  const ShowMaterialBannerExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ScaffoldMessengerState Sample')),
        body: const Center(child: ShowMaterialBannerExample()),
      ),
    );
  }
}

class ShowMaterialBannerExample extends StatelessWidget {
  const ShowMaterialBannerExample({super.key});

  @override
  Widget build(BuildContext context) {
    return OutlinedButton(
      onPressed: () {
        ScaffoldMessenger.of(context).showMaterialBanner(
          const MaterialBanner(
            content: Text('This is a MaterialBanner'),
            actions: <Widget>[TextButton(onPressed: null, child: Text('DISMISS'))],
          ),
        );
      },
      child: const Text('Show MaterialBanner'),
    );
  }
}
