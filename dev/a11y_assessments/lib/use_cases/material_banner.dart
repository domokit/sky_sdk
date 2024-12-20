// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class MaterialBannerUseCase extends UseCase {
  @override
  String get name => 'MaterialBanner';

  @override
  String get route => '/material_banner';

  @override
  Widget build(BuildContext context) => const MainWidget();
}

class MainWidget extends StatefulWidget {
  const MainWidget({super.key});

  @override
  State<MainWidget> createState() => MainWidgetState();
}

class MainWidgetState extends State<MainWidget> {
  final FocusNode dismissButtonFocusNode = FocusNode();
  final FocusNode showButtonFocusNode = FocusNode();

  String pageTitle = getUseCaseName(MaterialBannerUseCase());

  @override
  void dispose() {
    dismissButtonFocusNode.dispose();
    showButtonFocusNode.dispose();
    super.dispose();
  }

  void hideBanner() {
    ScaffoldMessenger.of(context).hideCurrentMaterialBanner();
    showButtonFocusNode.requestFocus();
  }

  void showBanner() {
    ScaffoldMessenger.of(context).showMaterialBanner(
      MaterialBanner(
        padding: const EdgeInsets.all(20),
        content: const Text('Hello, I am a Material Banner'),
        leading: const Icon(Icons.agriculture_outlined),
        backgroundColor: Colors.yellowAccent,
        actions: <Widget>[
          TextButton(
            focusNode: dismissButtonFocusNode,
            onPressed: hideBanner,
            child: const Text('DISMISS'),
          ),
        ],
      ),
    );
    dismissButtonFocusNode.requestFocus();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo')),
      ),
      body: Center(
        child: ElevatedButton(
          focusNode: showButtonFocusNode,
          onPressed: showBanner,
          child: const Text('Show a MaterialBanner'),
        ),
      ),
    );
  }
}
