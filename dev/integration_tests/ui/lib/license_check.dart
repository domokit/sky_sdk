// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

/// This application does nothing but show a screen with the flutter package
/// license in it.
void main() {
  enableFlutterDriverExtension();
  runApp(const ShowLicenses());
}

class ShowLicenses extends StatelessWidget {
  const ShowLicenses({super.key});

  Widget _buildTestResultWidget(BuildContext context, AsyncSnapshot<List<LicenseEntry>> snapshot) {
    final List<LicenseEntry> entries = snapshot.data ?? <LicenseEntry>[];
    String flutterPackage = '';
    final List<String> flutterParagraphs = <String>[];
    String enginePackage = '';
    final List<String> engineParagraphs = <String>[];
    for (final LicenseEntry entry in entries) {
      if (entry.packages.contains('flutter')) {
        flutterPackage = 'flutter';
        flutterParagraphs.addAll(
          entry.paragraphs.map<String>((LicenseParagraph para) => para.text),
        );
      }
      if (entry.packages.contains('engine')) {
        enginePackage = 'engine';
        engineParagraphs.addAll(entry.paragraphs.map<String>((LicenseParagraph para) => para.text));
      }
    }

    final List<Widget> result = <Widget>[];
    result.addAll(<Widget>[
      const Text('License Check Test', key: ValueKey<String>('Header')),
      Text(flutterPackage, key: const ValueKey<String>('FlutterPackage')),
      Text(flutterParagraphs.join(' '), key: const ValueKey<String>('FlutterLicense')),
      Text('${flutterParagraphs.length}', key: const ValueKey<String>('FlutterCount')),
      Text(enginePackage, key: const ValueKey<String>('EnginePackage')),
      Text(engineParagraphs.join(' '), key: const ValueKey<String>('EngineLicense')),
      Text('${engineParagraphs.length}', key: const ValueKey<String>('EngineCount')),
    ]);

    return ListView(children: result);
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: FutureBuilder<List<LicenseEntry>>(
          initialData: const <LicenseEntry>[],
          builder: _buildTestResultWidget,
          future: LicenseRegistry.licenses.toList(),
        ),
      ),
    );
  }
}
