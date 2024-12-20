// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ScaffoldMessengerState.showSnackBar].

void main() => runApp(const ShowSnackBarExampleApp());

class ShowSnackBarExampleApp extends StatelessWidget {
  const ShowSnackBarExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ScaffoldMessengerState Sample')),
        body: const Center(child: ShowSnackBarExample()),
      ),
    );
  }
}

class ShowSnackBarExample extends StatelessWidget {
  const ShowSnackBarExample({super.key});

  @override
  Widget build(BuildContext context) {
    return OutlinedButton(
      onPressed: () {
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(const SnackBar(content: Text('A SnackBar has been shown.')));
      },
      child: const Text('Show SnackBar'),
    );
  }
}
