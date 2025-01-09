// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_driver/driver_extension.dart';

void main() {
  enableFlutterDriverExtension();
  runMainApp();
}

void runMainApp() {
  runApp(const Center(child: Flavor()));
}

class Flavor extends StatefulWidget {
  const Flavor({super.key});

  @override
  State<Flavor> createState() => _FlavorState();
}

class _FlavorState extends State<Flavor> {
  String? _flavor;

  @override
  void initState() {
    super.initState();
    const MethodChannel('flavor').invokeMethod<String>('getFlavor').then((String? flavor) {
      setState(() {
        _flavor = flavor;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child:
          _flavor == null
              ? const Text('Awaiting flavor...')
              : Text(_flavor!, key: const ValueKey<String>('flavor')),
    );
  }
}
