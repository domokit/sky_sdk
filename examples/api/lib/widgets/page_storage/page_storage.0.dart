// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [PageStorage].

void main() => runApp(const PageStorageExampleApp());

class PageStorageExampleApp extends StatelessWidget {
  const PageStorageExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: MyHomePage());
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final List<Widget> pages = const <Widget>[
    ColorBoxPage(key: PageStorageKey<String>('pageOne')),
    ColorBoxPage(key: PageStorageKey<String>('pageTwo')),
  ];
  int currentTab = 0;
  final PageStorageBucket _bucket = PageStorageBucket();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Persistence Example')),
      body: PageStorage(bucket: _bucket, child: pages[currentTab]),
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: currentTab,
        onTap: (int index) {
          setState(() {
            currentTab = index;
          });
        },
        items: const <BottomNavigationBarItem>[
          BottomNavigationBarItem(icon: Icon(Icons.home), label: 'page 1'),
          BottomNavigationBarItem(icon: Icon(Icons.settings), label: 'page2'),
        ],
      ),
    );
  }
}

class ColorBoxPage extends StatelessWidget {
  const ColorBoxPage({super.key});

  @override
  Widget build(BuildContext context) {
    return ListView.builder(
      itemExtent: 250.0,
      itemBuilder:
          (BuildContext context, int index) => Container(
            padding: const EdgeInsets.all(10.0),
            child: Material(
              color: index.isEven ? Colors.cyan : Colors.deepOrange,
              child: Center(child: Text(index.toString())),
            ),
          ),
    );
  }
}
