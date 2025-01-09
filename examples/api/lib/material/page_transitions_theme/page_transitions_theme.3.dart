// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for the default Android U page transition theme
/// [FadeForwardsPageTransitionsBuilder]. Tapping each list tile navigates to
/// a second page, which slides in from right to left while fading in.
/// Simultaneously, the first page slides out in the same direction while
/// fading out.

void main() => runApp(const PageTransitionsThemeApp());

class PageTransitionsThemeApp extends StatelessWidget {
  const PageTransitionsThemeApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        pageTransitionsTheme: PageTransitionsTheme(
          builders: Map<TargetPlatform, PageTransitionsBuilder>.fromIterable(
            TargetPlatform.values,
            value: (_) => const FadeForwardsPageTransitionsBuilder(),
          ),
        ),
      ),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatelessWidget {
  const HomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        leading: IconButton(icon: const Icon(Icons.dehaze), onPressed: () {}),
        actions: <Widget>[
          IconButton(icon: const Icon(Icons.search), onPressed: () {}),
          IconButton(icon: const Icon(Icons.more_vert), onPressed: () {}),
        ],
      ),
      body: Column(
        children: <Widget>[
          Text('Messages', style: Theme.of(context).textTheme.headlineLarge),
          Expanded(
            child: Padding(
              padding: const EdgeInsets.all(20.0),
              child: Card(
                clipBehavior: Clip.antiAlias,
                elevation: 0,
                color: Theme.of(context).colorScheme.surfaceContainerLowest,
                child: ListView(
                  children: List<Widget>.generate(Colors.primaries.length, (int index) {
                    final Text kittenName = Text('Kitten $index');
                    final CircleAvatar avatar = CircleAvatar(
                      backgroundColor: Colors.primaries[index],
                    );
                    final String message =
                        index.isEven
                            ? 'Hello hooman! My name is Kitten $index'
                            : "What's up hooman! My name is Kitten $index";
                    return ListTile(
                      leading: avatar,
                      title: kittenName,
                      subtitle: Text(message),
                      trailing: Text('$index seconds ago'),
                      onTap: () {
                        Navigator.of(context).push(
                          MaterialPageRoute<SecondPage>(
                            builder:
                                (BuildContext context) => SecondPage(
                                  kittenName: kittenName,
                                  avatar: avatar,
                                  message: message,
                                ),
                          ),
                        );
                      },
                    );
                  }),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class SecondPage extends StatelessWidget {
  const SecondPage({
    super.key,
    required this.kittenName,
    required this.avatar,
    required this.message,
  });
  final Text kittenName;
  final CircleAvatar avatar;
  final String message;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        leading: const BackButton(),
        title: kittenName,
        centerTitle: false,
        actions: <Widget>[
          IconButton(icon: const Icon(Icons.search), onPressed: () {}),
          IconButton(icon: const Icon(Icons.more_vert), onPressed: () {}),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(20.0),
        child: IntrinsicHeight(
          child: Row(
            children: <Widget>[
              avatar,
              ConstrainedBox(
                constraints: const BoxConstraints(minHeight: 50),
                child: Card(
                  elevation: 0.0,
                  shape: const RoundedRectangleBorder(
                    borderRadius: BorderRadius.only(
                      topLeft: Radius.circular(20),
                      topRight: Radius.circular(20),
                      bottomLeft: Radius.circular(5),
                      bottomRight: Radius.circular(20),
                    ),
                  ),
                  color: Theme.of(context).colorScheme.surfaceContainerLowest,
                  child: Center(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 15.0),
                      child: Text(message),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
