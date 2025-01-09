// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'project.dart';

class BasicProject extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter
  ''';

  @override
  final String main = r'''
  import 'dart:async';

  import 'package:flutter/material.dart';

  Future<void> main() async {
    while (true) {
      runApp(MyApp());
      await Future.delayed(const Duration(milliseconds: 50));
    }
  }

  class MyApp extends StatelessWidget {
    @override
    Widget build(BuildContext context) {
      topLevelFunction();
      return MaterialApp( // BUILD BREAKPOINT
        title: 'Flutter Demo',
        home: Container(),
      );
    }
  }

  topLevelFunction() {
    print("topLevelFunction"); // TOP LEVEL BREAKPOINT
  }
  ''';

  Uri get buildMethodBreakpointUri => mainDart;
  int get buildMethodBreakpointLine => lineContaining(main, '// BUILD BREAKPOINT');

  Uri get topLevelFunctionBreakpointUri => mainDart;
  int get topLevelFunctionBreakpointLine => lineContaining(main, '// TOP LEVEL BREAKPOINT');
}

/// A project that throws multiple exceptions during Widget builds.
///
/// A repro for the issue at https://github.com/Dart-Code/Dart-Code/issues/3448
/// where Hot Restart could become stuck on exceptions and never complete.
class BasicProjectThatThrows extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter
  ''';

  @override
  final String main = r'''
  import 'package:flutter/material.dart';

  void a() {
    throw Exception('a');
  }

  void b() {
    try {
      a();
    } catch (e) {
      throw Exception('b');
    }
  }

  void c() {
    try {
      b();
    } catch (e) {
      throw Exception('c');
    }
  }

  void main() {
    runApp(App());
  }

  class App extends StatelessWidget {
    @override
    Widget build(BuildContext context) {
      c();
      return MaterialApp(
        debugShowCheckedModeBanner: false,
        title: 'Study Flutter',
        theme: ThemeData(
          primarySwatch: Colors.green,
        ),
        home: Container(),
      );
    }
  }
  ''';
}

class BasicProjectWithTimelineTraces extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter
  ''';

  @override
  final String main = r'''
  import 'dart:async';
  import 'dart:developer';

  import 'package:flutter/material.dart';

  Future<void> main() async {
    while (true) {
      runApp(MyApp());
      await Future.delayed(const Duration(milliseconds: 50));
      Timeline.instantSync('main');
    }
  }

  class MyApp extends StatelessWidget {
    @override
    Widget build(BuildContext context) {
      topLevelFunction();
      return MaterialApp( // BUILD BREAKPOINT
        title: 'Flutter Demo',
        home: Container(),
      );
    }
  }

  topLevelFunction() {
    print("topLevelFunction"); // TOP LEVEL BREAKPOINT
  }
  ''';
}

class BasicProjectWithFlutterGen extends Project {
  @override
  final String generatedFile = '''
    String x = "a";
  ''';

  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter

  flutter:
    generate: true
  ''';

  @override
  final String main = r'''
  import 'dart:async';
  import 'package:flutter_gen/flutter_gen.dart';

  void main() {}
  ''';
}

class BasicProjectWithUnaryMain extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0
  dependencies:
    flutter:
      sdk: flutter
  ''';

  @override
  final String main = r'''
  import 'dart:async';
  import 'package:flutter/material.dart';
  Future<void> main(List<String> args) async {
    while (true) {
      runApp(MyApp());
      await Future.delayed(const Duration(milliseconds: 50));
    }
  }
  class MyApp extends StatelessWidget {
    @override
    Widget build(BuildContext context) {
      topLevelFunction();
      return MaterialApp( // BUILD BREAKPOINT
        title: 'Flutter Demo',
        home: Container(),
      );
    }
  }
  topLevelFunction() {
    print("topLevelFunction"); // TOP LEVEL BREAKPOINT
  }
  ''';
}
