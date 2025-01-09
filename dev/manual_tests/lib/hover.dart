// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  runApp(const MaterialApp(title: 'Hover Demo', home: HoverDemo()));
}

class HoverDemo extends StatefulWidget {
  const HoverDemo({super.key});

  @override
  State<HoverDemo> createState() => _HoverDemoState();
}

class _HoverDemoState extends State<HoverDemo> {
  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final ButtonStyle overrideFocusColor = ButtonStyle(
      overlayColor: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
        return states.contains(MaterialState.focused)
            ? Colors.deepOrangeAccent
            : Colors.transparent;
      }),
    );

    return DefaultTextStyle(
      style: textTheme.headlineMedium!,
      child: Scaffold(
        appBar: AppBar(title: const Text('Hover Demo')),
        floatingActionButton: FloatingActionButton(child: const Text('+'), onPressed: () {}),
        body: Center(
          child: Builder(
            builder: (BuildContext context) {
              return Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  Row(
                    children: <Widget>[
                      ElevatedButton(
                        onPressed: () => print('Button pressed.'),
                        style: overrideFocusColor,
                        child: const Text('Button'),
                      ),
                      TextButton(
                        onPressed: () => print('Button pressed.'),
                        style: overrideFocusColor,
                        child: const Text('Button'),
                      ),
                      IconButton(
                        onPressed: () => print('Button pressed'),
                        icon: const Icon(Icons.access_alarm),
                        focusColor: Colors.deepOrangeAccent,
                      ),
                    ],
                  ),
                  const Padding(
                    padding: EdgeInsets.all(8.0),
                    child: TextField(
                      decoration: InputDecoration(labelText: 'Enter Text', filled: true),
                    ),
                  ),
                  const Padding(
                    padding: EdgeInsets.all(8.0),
                    child: TextField(
                      decoration: InputDecoration(
                        border: OutlineInputBorder(),
                        labelText: 'Enter Text',
                        filled: false,
                      ),
                    ),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );
  }
}
