// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [InheritedModel].

enum LogoAspect { backgroundColor, large }

void main() => runApp(const InheritedModelApp());

class InheritedModelApp extends StatelessWidget {
  const InheritedModelApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: InheritedModelExample());
  }
}

class LogoModel extends InheritedModel<LogoAspect> {
  const LogoModel({super.key, this.backgroundColor, this.large, required super.child});

  final Color? backgroundColor;
  final bool? large;

  static Color? backgroundColorOf(BuildContext context) {
    return InheritedModel.inheritFrom<LogoModel>(
      context,
      aspect: LogoAspect.backgroundColor,
    )?.backgroundColor;
  }

  static bool sizeOf(BuildContext context) {
    return InheritedModel.inheritFrom<LogoModel>(context, aspect: LogoAspect.large)?.large ?? false;
  }

  @override
  bool updateShouldNotify(LogoModel oldWidget) {
    return backgroundColor != oldWidget.backgroundColor || large != oldWidget.large;
  }

  @override
  bool updateShouldNotifyDependent(LogoModel oldWidget, Set<LogoAspect> dependencies) {
    if (backgroundColor != oldWidget.backgroundColor &&
        dependencies.contains(LogoAspect.backgroundColor)) {
      return true;
    }
    if (large != oldWidget.large && dependencies.contains(LogoAspect.large)) {
      return true;
    }
    return false;
  }
}

class InheritedModelExample extends StatefulWidget {
  const InheritedModelExample({super.key});

  @override
  State<InheritedModelExample> createState() => _InheritedModelExampleState();
}

class _InheritedModelExampleState extends State<InheritedModelExample> {
  bool large = false;
  Color color = Colors.blue;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('InheritedModel Sample')),
      body: Column(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: <Widget>[
          Center(
            child: LogoModel(
              backgroundColor: color,
              large: large,
              child: const BackgroundWidget(child: LogoWidget()),
            ),
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: <Widget>[
              ElevatedButton(
                onPressed: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('Rebuilt Background'),
                      duration: Duration(milliseconds: 500),
                    ),
                  );
                  setState(() {
                    if (color == Colors.blue) {
                      color = Colors.red;
                    } else {
                      color = Colors.blue;
                    }
                  });
                },
                child: const Text('Update background'),
              ),
              ElevatedButton(
                onPressed: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('Rebuilt LogoWidget'),
                      duration: Duration(milliseconds: 500),
                    ),
                  );
                  setState(() {
                    large = !large;
                  });
                },
                child: const Text('Resize Logo'),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class BackgroundWidget extends StatelessWidget {
  const BackgroundWidget({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    final Color color = LogoModel.backgroundColorOf(context)!;

    return AnimatedContainer(
      padding: const EdgeInsets.all(12.0),
      color: color,
      duration: const Duration(seconds: 2),
      curve: Curves.fastOutSlowIn,
      child: child,
    );
  }
}

class LogoWidget extends StatelessWidget {
  const LogoWidget({super.key});

  @override
  Widget build(BuildContext context) {
    final bool largeLogo = LogoModel.sizeOf(context);

    return AnimatedContainer(
      padding: const EdgeInsets.all(20.0),
      duration: const Duration(seconds: 2),
      curve: Curves.fastLinearToSlowEaseIn,
      alignment: Alignment.center,
      child: FlutterLogo(size: largeLogo ? 200.0 : 100.0),
    );
  }
}
