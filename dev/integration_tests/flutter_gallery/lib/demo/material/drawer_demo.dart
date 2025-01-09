// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

const String _kAsset0 = 'people/square/trevor.png';
const String _kAsset1 = 'people/square/stella.png';
const String _kAsset2 = 'people/square/sandra.png';
const String _kGalleryAssetsPackage = 'flutter_gallery_assets';

class DrawerDemo extends StatefulWidget {
  const DrawerDemo({super.key});

  static const String routeName = '/material/drawer';

  @override
  State<DrawerDemo> createState() => _DrawerDemoState();
}

class _DrawerDemoState extends State<DrawerDemo> with TickerProviderStateMixin {
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();

  static const List<String> _drawerContents = <String>['A', 'B', 'C', 'D', 'E'];

  static final Animatable<Offset> _drawerDetailsTween = Tween<Offset>(
    begin: const Offset(0.0, -1.0),
    end: Offset.zero,
  ).chain(CurveTween(curve: Curves.fastOutSlowIn));

  late AnimationController _controller;
  late Animation<double> _drawerContentsOpacity;
  late Animation<Offset> _drawerDetailsPosition;
  bool _showDrawerContents = true;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(vsync: this, duration: const Duration(milliseconds: 200));
    _drawerContentsOpacity = CurvedAnimation(
      parent: ReverseAnimation(_controller),
      curve: Curves.fastOutSlowIn,
    );
    _drawerDetailsPosition = _controller.drive(_drawerDetailsTween);
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  IconData? _backIcon() {
    switch (Theme.of(context).platform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return Icons.arrow_back;
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        return Icons.arrow_back_ios;
    }
  }

  void _showNotImplementedMessage() {
    Navigator.pop(context); // Dismiss the drawer.
    ScaffoldMessenger.of(
      context,
    ).showSnackBar(const SnackBar(content: Text("The drawer's items don't do anything")));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawerDragStartBehavior: DragStartBehavior.down,
      key: _scaffoldKey,
      appBar: AppBar(
        leading: IconButton(
          icon: Icon(_backIcon()),
          alignment: Alignment.centerLeft,
          tooltip: 'Back',
          onPressed: () {
            Navigator.pop(context);
          },
        ),
        title: const Text('Navigation drawer'),
        actions: <Widget>[MaterialDemoDocumentationButton(DrawerDemo.routeName)],
      ),
      drawer: Drawer(
        child: Column(
          children: <Widget>[
            UserAccountsDrawerHeader(
              accountName: const Text('Trevor Widget'),
              accountEmail: const Text('trevor.widget@example.com'),
              currentAccountPicture: const CircleAvatar(
                backgroundImage: AssetImage(_kAsset0, package: _kGalleryAssetsPackage),
              ),
              otherAccountsPictures: <Widget>[
                GestureDetector(
                  dragStartBehavior: DragStartBehavior.down,
                  onTap: () {
                    _onOtherAccountsTap(context);
                  },
                  child: Semantics(
                    label: 'Switch to Account B',
                    child: const CircleAvatar(
                      backgroundImage: AssetImage(_kAsset1, package: _kGalleryAssetsPackage),
                    ),
                  ),
                ),
                GestureDetector(
                  dragStartBehavior: DragStartBehavior.down,
                  onTap: () {
                    _onOtherAccountsTap(context);
                  },
                  child: Semantics(
                    label: 'Switch to Account C',
                    child: const CircleAvatar(
                      backgroundImage: AssetImage(_kAsset2, package: _kGalleryAssetsPackage),
                    ),
                  ),
                ),
              ],
              margin: EdgeInsets.zero,
              onDetailsPressed: () {
                _showDrawerContents = !_showDrawerContents;
                if (_showDrawerContents) {
                  _controller.reverse();
                } else {
                  _controller.forward();
                }
              },
            ),
            MediaQuery.removePadding(
              context: context,
              // DrawerHeader consumes top MediaQuery padding.
              removeTop: true,
              child: Expanded(
                child: ListView(
                  dragStartBehavior: DragStartBehavior.down,
                  padding: const EdgeInsets.only(top: 8.0),
                  children: <Widget>[
                    Stack(
                      children: <Widget>[
                        // The initial contents of the drawer.
                        FadeTransition(
                          opacity: _drawerContentsOpacity,
                          child: Column(
                            mainAxisSize: MainAxisSize.min,
                            crossAxisAlignment: CrossAxisAlignment.stretch,
                            children:
                                _drawerContents.map<Widget>((String id) {
                                  return ListTile(
                                    leading: CircleAvatar(child: Text(id)),
                                    title: Text('Drawer item $id'),
                                    onTap: _showNotImplementedMessage,
                                  );
                                }).toList(),
                          ),
                        ),
                        // The drawer's "details" view.
                        SlideTransition(
                          position: _drawerDetailsPosition,
                          child: FadeTransition(
                            opacity: ReverseAnimation(_drawerContentsOpacity),
                            child: Column(
                              mainAxisSize: MainAxisSize.min,
                              crossAxisAlignment: CrossAxisAlignment.stretch,
                              children: <Widget>[
                                ListTile(
                                  leading: const Icon(Icons.add),
                                  title: const Text('Add account'),
                                  onTap: _showNotImplementedMessage,
                                ),
                                ListTile(
                                  leading: const Icon(Icons.settings),
                                  title: const Text('Manage accounts'),
                                  onTap: _showNotImplementedMessage,
                                ),
                              ],
                            ),
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
      body: Center(
        child: InkWell(
          onTap: () {
            _scaffoldKey.currentState!.openDrawer();
          },
          child: Semantics(
            button: true,
            label: 'Open drawer',
            excludeSemantics: true,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                Container(
                  width: 100.0,
                  height: 100.0,
                  decoration: const BoxDecoration(
                    shape: BoxShape.circle,
                    image: DecorationImage(
                      image: AssetImage(_kAsset0, package: _kGalleryAssetsPackage),
                    ),
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.only(top: 8.0),
                  child: Text(
                    'Tap here to open the drawer',
                    style: Theme.of(context).textTheme.titleMedium,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  void _onOtherAccountsTap(BuildContext context) {
    showDialog<void>(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: const Text('Account switching not implemented.'),
          actions: <Widget>[
            TextButton(
              child: const Text('OK'),
              onPressed: () {
                Navigator.pop(context);
              },
            ),
          ],
        );
      },
    );
  }
}
