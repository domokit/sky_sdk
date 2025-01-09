// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [ServicesBinding.handleRequestAppExit].

void main() {
  runApp(const ApplicationExitExample());
}

class ApplicationExitExample extends StatelessWidget {
  const ApplicationExitExample({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Scaffold(body: Body()));
  }
}

class Body extends StatefulWidget {
  const Body({super.key});

  @override
  State<Body> createState() => _BodyState();
}

class _BodyState extends State<Body> with WidgetsBindingObserver {
  bool _shouldExit = false;
  String lastResponse = 'No exit requested yet';

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  Future<void> _quit() async {
    final AppExitType exitType = _shouldExit ? AppExitType.required : AppExitType.cancelable;
    setState(() {
      lastResponse = 'App requesting ${exitType.name} exit';
    });
    await ServicesBinding.instance.exitApplication(exitType);
  }

  @override
  Future<AppExitResponse> didRequestAppExit() async {
    final AppExitResponse response = _shouldExit ? AppExitResponse.exit : AppExitResponse.cancel;
    setState(() {
      lastResponse = 'App responded ${response.name} to exit request';
    });
    return response;
  }

  void _radioChanged(bool? value) {
    value ??= true;
    if (_shouldExit == value) {
      return;
    }
    setState(() {
      _shouldExit = value!;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: SizedBox(
        width: 300,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            RadioListTile<bool>(
              title: const Text('Do Not Allow Exit'),
              groupValue: _shouldExit,
              value: false,
              onChanged: _radioChanged,
            ),
            RadioListTile<bool>(
              title: const Text('Allow Exit'),
              groupValue: _shouldExit,
              value: true,
              onChanged: _radioChanged,
            ),
            const SizedBox(height: 30),
            ElevatedButton(onPressed: _quit, child: const Text('Quit')),
            const SizedBox(height: 30),
            Text(lastResponse),
          ],
        ),
      ),
    );
  }
}
