// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ActionIconTheme].

void main() {
  runApp(const ActionIconThemeExampleApp());
}

class _CustomEndDrawerIcon extends StatelessWidget {
  const _CustomEndDrawerIcon();

  @override
  Widget build(BuildContext context) {
    final MaterialLocalizations localization = MaterialLocalizations.of(context);
    return Icon(Icons.more_horiz, semanticLabel: localization.openAppDrawerTooltip);
  }
}

class _CustomDrawerIcon extends StatelessWidget {
  const _CustomDrawerIcon();

  @override
  Widget build(BuildContext context) {
    final MaterialLocalizations localization = MaterialLocalizations.of(context);
    return Icon(Icons.segment, semanticLabel: localization.openAppDrawerTooltip);
  }
}

class ActionIconThemeExampleApp extends StatelessWidget {
  const ActionIconThemeExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        actionIconTheme: ActionIconThemeData(
          backButtonIconBuilder: (BuildContext context) {
            return const Icon(Icons.arrow_back_ios_new_rounded);
          },
          drawerButtonIconBuilder: (BuildContext context) {
            return const _CustomDrawerIcon();
          },
          endDrawerButtonIconBuilder: (BuildContext context) {
            return const _CustomEndDrawerIcon();
          },
        ),
      ),
      home: const MyHomePage(title: 'Flutter Demo Home Page'),
    );
  }
}

class MyHomePage extends StatelessWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(title)),
      drawer: Drawer(
        child: Column(
          children: <Widget>[TextButton(child: const Text('Drawer Item'), onPressed: () {})],
        ),
      ),
      body: const Center(child: NextPageButton()),
    );
  }
}

class NextPageButton extends StatelessWidget {
  const NextPageButton({super.key});

  @override
  Widget build(BuildContext context) {
    return ElevatedButton.icon(
      onPressed: () {
        Navigator.of(context).push(
          MaterialPageRoute<MySecondPage>(
            builder: (BuildContext context) {
              return const MySecondPage();
            },
          ),
        );
      },
      icon: const Icon(Icons.arrow_forward),
      label: const Text('Next page'),
    );
  }
}

class MySecondPage extends StatelessWidget {
  const MySecondPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(appBar: AppBar(title: const Text('Second page')), endDrawer: const Drawer());
  }
}
