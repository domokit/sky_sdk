// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

/// Flutter code sample for [ErrorWidget].

void main() {
  // Set the ErrorWidget's builder before the app is started.
  ErrorWidget.builder = (FlutterErrorDetails details) {
    // If we're in debug mode, use the normal error widget which shows the error
    // message:
    if (kDebugMode) {
      return ErrorWidget(details.exception);
    }
    // In release builds, show a yellow-on-blue message instead:
    return ReleaseModeErrorWidget(details: details);
  };

  // Start the app.
  runApp(const ErrorWidgetExampleApp());
}

class ErrorWidgetExampleApp extends StatefulWidget {
  const ErrorWidgetExampleApp({super.key});

  @override
  State<ErrorWidgetExampleApp> createState() => _ErrorWidgetExampleAppState();
}

class _ErrorWidgetExampleAppState extends State<ErrorWidgetExampleApp> {
  bool throwError = false;

  @override
  Widget build(BuildContext context) {
    if (throwError) {
      // Since the error widget is only used during a build, in this contrived example,
      // we purposely throw an exception in a build function.
      return Builder(
        builder: (BuildContext context) {
          throw Exception('oh no, an error');
        },
      );
    } else {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('ErrorWidget Sample')),
          body: Center(
            child: TextButton(
              onPressed: () {
                setState(() {
                  throwError = true;
                });
              },
              child: const Text('Error Prone'),
            ),
          ),
        ),
      );
    }
  }
}

class ReleaseModeErrorWidget extends StatelessWidget {
  const ReleaseModeErrorWidget({super.key, required this.details});

  final FlutterErrorDetails details;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Text(
        'Error!\n${details.exception}',
        style: const TextStyle(color: Colors.yellow),
        textAlign: TextAlign.center,
        textDirection: TextDirection.ltr,
      ),
    );
  }
}
