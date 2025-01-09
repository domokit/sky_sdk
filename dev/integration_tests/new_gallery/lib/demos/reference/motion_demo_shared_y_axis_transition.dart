// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN sharedYAxisTransitionDemo

class SharedYAxisTransitionDemo extends StatefulWidget {
  const SharedYAxisTransitionDemo({super.key});

  @override
  State<SharedYAxisTransitionDemo> createState() => _SharedYAxisTransitionDemoState();
}

class _SharedYAxisTransitionDemoState extends State<SharedYAxisTransitionDemo>
    with SingleTickerProviderStateMixin {
  bool _isAlphabetical = false;
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(duration: const Duration(milliseconds: 300), vsync: this);
  }

  final ListView _recentList = ListView(
    // Adding [UniqueKey] to make sure the widget rebuilds when transitioning.
    key: UniqueKey(),
    children: <Widget>[for (int i = 0; i < 10; i++) _AlbumTile((i + 1).toString())],
  );

  final ListView _alphabeticalList = ListView(
    // Adding [UniqueKey] to make sure the widget rebuilds when transitioning.
    key: UniqueKey(),
    children: <Widget>[for (final String letter in _alphabet) _AlbumTile(letter)],
  );

  static const List<String> _alphabet = <String>['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'];

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Column(
          children: <Widget>[
            Text(localizations.demoSharedYAxisTitle),
            Text(
              '(${localizations.demoSharedYAxisDemoInstructions})',
              style: Theme.of(context).textTheme.titleSmall!.copyWith(color: Colors.white),
            ),
          ],
        ),
      ),
      body: Column(
        children: <Widget>[
          const SizedBox(height: 5),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: <Widget>[
              Padding(
                padding: const EdgeInsets.only(left: 15),
                child: Text(localizations.demoSharedYAxisAlbumCount),
              ),
              Padding(
                padding: const EdgeInsets.only(right: 7),
                child: InkWell(
                  customBorder: const RoundedRectangleBorder(
                    borderRadius: BorderRadius.all(Radius.circular(4)),
                  ),
                  onTap: () {
                    if (!_isAlphabetical) {
                      _controller.reset();
                      _controller.animateTo(0.5);
                    } else {
                      _controller.animateTo(1);
                    }
                    setState(() {
                      _isAlphabetical = !_isAlphabetical;
                    });
                  },
                  child: Row(
                    children: <Widget>[
                      Text(
                        _isAlphabetical
                            ? localizations.demoSharedYAxisAlphabeticalSortTitle
                            : localizations.demoSharedYAxisRecentSortTitle,
                      ),
                      RotationTransition(
                        turns: Tween<double>(begin: 0.0, end: 1.0).animate(_controller.view),
                        child: const Icon(Icons.arrow_drop_down),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 10),
          Expanded(
            child: PageTransitionSwitcher(
              reverse: _isAlphabetical,
              transitionBuilder: (
                Widget child,
                Animation<double> animation,
                Animation<double> secondaryAnimation,
              ) {
                return SharedAxisTransition(
                  animation: animation,
                  secondaryAnimation: secondaryAnimation,
                  transitionType: SharedAxisTransitionType.vertical,
                  child: child,
                );
              },
              child: _isAlphabetical ? _alphabeticalList : _recentList,
            ),
          ),
        ],
      ),
    );
  }
}

class _AlbumTile extends StatelessWidget {
  const _AlbumTile(this._title);
  final String _title;

  @override
  Widget build(BuildContext context) {
    final Random randomNumberGenerator = Random();
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Column(
      children: <Widget>[
        ListTile(
          leading: Container(
            height: 60,
            width: 60,
            decoration: const BoxDecoration(
              borderRadius: BorderRadius.all(Radius.circular(4)),
              color: Colors.grey,
            ),
            child: Padding(
              padding: const EdgeInsets.all(6),
              child: Image.asset(
                'placeholders/placeholder_image.png',
                package: 'flutter_gallery_assets',
              ),
            ),
          ),
          title: Text('${localizations.demoSharedYAxisAlbumTileTitle} $_title'),
          subtitle: Text(localizations.demoSharedYAxisAlbumTileSubtitle),
          trailing: Text(
            '${randomNumberGenerator.nextInt(50) + 10} '
            '${localizations.demoSharedYAxisAlbumTileDurationUnit}',
          ),
        ),
        const Divider(height: 20, thickness: 1),
      ],
    );
  }
}

// END sharedYAxisTransitionDemo
