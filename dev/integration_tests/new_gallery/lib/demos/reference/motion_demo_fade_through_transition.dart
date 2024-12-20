// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN fadeThroughTransitionDemo

class FadeThroughTransitionDemo extends StatefulWidget {
  const FadeThroughTransitionDemo({super.key});

  @override
  State<FadeThroughTransitionDemo> createState() => _FadeThroughTransitionDemoState();
}

class _FadeThroughTransitionDemoState extends State<FadeThroughTransitionDemo> {
  int _pageIndex = 0;

  final List<Widget> _pageList = <Widget>[_AlbumsPage(), _PhotosPage(), _SearchPage()];

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Column(
          children: <Widget>[
            Text(localizations.demoFadeThroughTitle),
            Text(
              '(${localizations.demoFadeThroughDemoInstructions})',
              style: Theme.of(context).textTheme.titleSmall!.copyWith(color: Colors.white),
            ),
          ],
        ),
      ),
      body: PageTransitionSwitcher(
        transitionBuilder: (
          Widget child,
          Animation<double> animation,
          Animation<double> secondaryAnimation,
        ) {
          return FadeThroughTransition(
            animation: animation,
            secondaryAnimation: secondaryAnimation,
            child: child,
          );
        },
        child: _pageList[_pageIndex],
      ),
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: _pageIndex,
        onTap: (int selectedIndex) {
          setState(() {
            _pageIndex = selectedIndex;
          });
        },
        items: <BottomNavigationBarItem>[
          BottomNavigationBarItem(
            icon: const Icon(Icons.photo_library),
            label: localizations.demoFadeThroughAlbumsDestination,
          ),
          BottomNavigationBarItem(
            icon: const Icon(Icons.photo),
            label: localizations.demoFadeThroughPhotosDestination,
          ),
          BottomNavigationBarItem(
            icon: const Icon(Icons.search),
            label: localizations.demoFadeThroughSearchDestination,
          ),
        ],
      ),
    );
  }
}

class _ExampleCard extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    final TextTheme textTheme = Theme.of(context).textTheme;

    return Expanded(
      child: Card(
        child: Stack(
          children: <Widget>[
            Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: <Widget>[
                Expanded(
                  child: ColoredBox(
                    color: Colors.black26,
                    child: Padding(
                      padding: const EdgeInsets.all(30),
                      child: Ink.image(
                        image: const AssetImage(
                          'placeholders/placeholder_image.png',
                          package: 'flutter_gallery_assets',
                        ),
                      ),
                    ),
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.all(8),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Text(
                        localizations.demoFadeThroughTextPlaceholder,
                        style: textTheme.bodyLarge,
                      ),
                      Text(
                        localizations.demoFadeThroughTextPlaceholder,
                        style: textTheme.bodySmall,
                      ),
                    ],
                  ),
                ),
              ],
            ),
            InkWell(splashColor: Colors.black38, onTap: () {}),
          ],
        ),
      ),
    );
  }
}

class _AlbumsPage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(
      children: <Widget>[
        ...List<Widget>.generate(
          3,
          (int index) => Expanded(
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: <Widget>[_ExampleCard(), _ExampleCard()],
            ),
          ),
        ),
      ],
    );
  }
}

class _PhotosPage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(children: <Widget>[_ExampleCard(), _ExampleCard()]);
  }
}

class _SearchPage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations? localizations = GalleryLocalizations.of(context);

    return ListView.builder(
      itemBuilder: (BuildContext context, int index) {
        return ListTile(
          leading: Image.asset(
            'placeholders/avatar_logo.png',
            package: 'flutter_gallery_assets',
            width: 40,
          ),
          title: Text('${localizations!.demoMotionListTileTitle} ${index + 1}'),
          subtitle: Text(localizations.demoMotionPlaceholderSubtitle),
        );
      },
      itemCount: 10,
    );
  }
}

// END fadeThroughTransitionDemo
