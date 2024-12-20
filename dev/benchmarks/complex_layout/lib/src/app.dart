// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart' show timeDilation;

enum ScrollMode { complex, tile }

class ComplexLayoutApp extends StatefulWidget {
  const ComplexLayoutApp({super.key, this.badScroll = false});

  final bool badScroll;

  @override
  ComplexLayoutAppState createState() => ComplexLayoutAppState();

  static ComplexLayoutAppState? of(BuildContext context) =>
      context.findAncestorStateOfType<ComplexLayoutAppState>();
}

class ComplexLayoutAppState extends State<ComplexLayoutApp> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: lightTheme ? ThemeData.light() : ThemeData.dark(),
      title: 'Advanced Layout',
      home:
          scrollMode == ScrollMode.complex
              ? ComplexLayout(badScroll: widget.badScroll)
              : const TileScrollLayout(),
    );
  }

  bool _lightTheme = true;
  bool get lightTheme => _lightTheme;
  set lightTheme(bool value) {
    setState(() {
      _lightTheme = value;
    });
  }

  ScrollMode _scrollMode = ScrollMode.complex;
  ScrollMode get scrollMode => _scrollMode;
  set scrollMode(ScrollMode mode) {
    setState(() {
      _scrollMode = mode;
    });
  }

  void toggleAnimationSpeed() {
    setState(() {
      timeDilation = (timeDilation != 1.0) ? 1.0 : 5.0;
    });
  }
}

class TileScrollLayout extends StatelessWidget {
  const TileScrollLayout({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Tile Scrolling Layout')),
      body: ListView.builder(
        key: const Key('tiles-scroll'),
        itemCount: 200,
        itemBuilder: (BuildContext context, int index) {
          return Padding(
            padding: const EdgeInsets.all(5.0),
            child: Material(
              elevation: (index % 5 + 1).toDouble(),
              color: Colors.white,
              child: const IconBar(),
            ),
          );
        },
      ),
      drawer: const GalleryDrawer(),
    );
  }
}

class ComplexLayout extends StatefulWidget {
  const ComplexLayout({super.key, required this.badScroll});

  final bool badScroll;

  @override
  ComplexLayoutState createState() => ComplexLayoutState();

  static ComplexLayoutState? of(BuildContext context) =>
      context.findAncestorStateOfType<ComplexLayoutState>();
}

class ComplexLayoutState extends State<ComplexLayout> {
  @override
  Widget build(BuildContext context) {
    Widget body = ListView.builder(
      key: const Key('complex-scroll'), // this key is used by the driver test
      controller: ScrollController(), // So that the scroll offset can be tracked
      itemCount: widget.badScroll ? 500 : null,
      shrinkWrap: widget.badScroll,
      itemBuilder: (BuildContext context, int index) {
        if (index.isEven) {
          return FancyImageItem(index, key: PageStorageKey<int>(index));
        } else {
          return FancyGalleryItem(index, key: PageStorageKey<int>(index));
        }
      },
    );
    if (widget.badScroll) {
      body = ListView(key: const Key('complex-scroll-bad'), children: <Widget>[body]);
    }

    return Scaffold(
      appBar: AppBar(
        title: const Text('Advanced Layout'),
        actions: <Widget>[
          IconButton(
            icon: const Icon(Icons.create),
            tooltip: 'Search',
            onPressed: () {
              print('Pressed search');
            },
          ),
          const TopBarMenu(),
        ],
      ),
      body: Column(children: <Widget>[Expanded(child: body), const BottomBar()]),
      drawer: const GalleryDrawer(),
    );
  }
}

class TopBarMenu extends StatelessWidget {
  const TopBarMenu({super.key});

  @override
  Widget build(BuildContext context) {
    return PopupMenuButton<String>(
      onSelected: (String value) {
        print('Selected: $value');
      },
      itemBuilder:
          (BuildContext context) => <PopupMenuItem<String>>[
            const PopupMenuItem<String>(
              value: 'Friends',
              child: MenuItemWithIcon(Icons.people, 'Friends', '5 new'),
            ),
            const PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.event, 'Events', '12 upcoming'),
            ),
            const PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.group, 'Groups', '14'),
            ),
            const PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.image, 'Pictures', '12'),
            ),
            const PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.near_me, 'Nearby', '33'),
            ),
            const PopupMenuItem<String>(
              value: 'Friends',
              child: MenuItemWithIcon(Icons.people, 'Friends', '5'),
            ),
            const PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.event, 'Events', '12'),
            ),
            const PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.group, 'Groups', '14'),
            ),
            const PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.image, 'Pictures', '12'),
            ),
            const PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.near_me, 'Nearby', '33'),
            ),
          ],
    );
  }
}

class MenuItemWithIcon extends StatelessWidget {
  const MenuItemWithIcon(this.icon, this.title, this.subtitle, {super.key});

  final IconData icon;
  final String title;
  final String subtitle;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: <Widget>[
        Icon(icon),
        Padding(padding: const EdgeInsets.only(left: 8.0, right: 8.0), child: Text(title)),
        Text(subtitle, style: Theme.of(context).textTheme.bodySmall),
      ],
    );
  }
}

class FancyImageItem extends StatelessWidget {
  const FancyImageItem(this.index, {super.key});

  final int index;

  @override
  Widget build(BuildContext context) {
    return ListBody(
      children: <Widget>[
        UserHeader('Ali Connors $index'),
        const ItemDescription(),
        const ItemImageBox(),
        const InfoBar(),
        const Padding(padding: EdgeInsets.symmetric(horizontal: 8.0), child: Divider()),
        const IconBar(),
        const FatDivider(),
      ],
    );
  }
}

class FancyGalleryItem extends StatelessWidget {
  const FancyGalleryItem(this.index, {super.key});

  final int index;
  @override
  Widget build(BuildContext context) {
    return ListBody(
      children: <Widget>[
        const UserHeader('Ali Connors'),
        ItemGalleryBox(index),
        const InfoBar(),
        const Padding(padding: EdgeInsets.symmetric(horizontal: 8.0), child: Divider()),
        const IconBar(),
        const FatDivider(),
      ],
    );
  }
}

class InfoBar extends StatelessWidget {
  const InfoBar({super.key});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: <Widget>[
          const MiniIconWithText(Icons.thumb_up, '42'),
          Text('3 Comments', style: Theme.of(context).textTheme.bodySmall),
        ],
      ),
    );
  }
}

class IconBar extends StatelessWidget {
  const IconBar({super.key});

  @override
  Widget build(BuildContext context) {
    return const Padding(
      padding: EdgeInsets.only(left: 16.0, right: 16.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: <Widget>[
          IconWithText(Icons.thumb_up, 'Like'),
          IconWithText(Icons.comment, 'Comment'),
          IconWithText(Icons.share, 'Share'),
        ],
      ),
    );
  }
}

class IconWithText extends StatelessWidget {
  const IconWithText(this.icon, this.title, {super.key});

  final IconData icon;
  final String title;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        IconButton(
          icon: Icon(icon),
          onPressed: () {
            print('Pressed $title button');
          },
        ),
        Text(title),
      ],
    );
  }
}

class MiniIconWithText extends StatelessWidget {
  const MiniIconWithText(this.icon, this.title, {super.key});

  final IconData icon;
  final String title;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        Padding(
          padding: const EdgeInsets.only(right: 8.0),
          child: Container(
            width: 16.0,
            height: 16.0,
            decoration: ShapeDecoration(
              color: Theme.of(context).primaryColor,
              shape: const CircleBorder(),
            ),
            child: Icon(icon, color: Colors.white, size: 12.0),
          ),
        ),
        Text(title, style: Theme.of(context).textTheme.bodySmall),
      ],
    );
  }
}

class FatDivider extends StatelessWidget {
  const FatDivider({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(height: 8.0, color: Theme.of(context).dividerColor);
  }
}

class UserHeader extends StatelessWidget {
  const UserHeader(this.userName, {super.key});

  final String userName;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          const Padding(
            padding: EdgeInsets.only(right: 8.0),
            child: Image(
              image: AssetImage('packages/flutter_gallery_assets/people/square/ali.png'),
              width: 32.0,
              height: 32.0,
            ),
          ),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: <Widget>[
                RichText(
                  text: TextSpan(
                    style: Theme.of(context).textTheme.bodyMedium,
                    children: <TextSpan>[
                      TextSpan(text: userName, style: const TextStyle(fontWeight: FontWeight.bold)),
                      const TextSpan(text: ' shared a new '),
                      const TextSpan(text: 'photo', style: TextStyle(fontWeight: FontWeight.bold)),
                    ],
                  ),
                ),
                Row(
                  children: <Widget>[
                    Text('Yesterday at 11:55 • ', style: Theme.of(context).textTheme.bodySmall),
                    Icon(
                      Icons.people,
                      size: 16.0,
                      color: Theme.of(context).textTheme.bodySmall!.color,
                    ),
                  ],
                ),
              ],
            ),
          ),
          const TopBarMenu(),
        ],
      ),
    );
  }
}

class ItemDescription extends StatelessWidget {
  const ItemDescription({super.key});

  @override
  Widget build(BuildContext context) {
    return const Padding(
      padding: EdgeInsets.all(8.0),
      child: Text(
        'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.',
      ),
    );
  }
}

class ItemImageBox extends StatelessWidget {
  const ItemImageBox({super.key});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Card(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            Stack(
              children: <Widget>[
                const SizedBox(
                  height: 230.0,
                  child: Image(
                    image: AssetImage(
                      'packages/flutter_gallery_assets/places/india_chettinad_silk_maker.png',
                    ),
                  ),
                ),
                Theme(
                  data: ThemeData.dark(),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.end,
                    children: <Widget>[
                      IconButton(
                        icon: const Icon(Icons.edit),
                        onPressed: () {
                          print('Pressed edit button');
                        },
                      ),
                      IconButton(
                        icon: const Icon(Icons.zoom_in),
                        onPressed: () {
                          print('Pressed zoom button');
                        },
                      ),
                    ],
                  ),
                ),
                Positioned(
                  bottom: 4.0,
                  left: 4.0,
                  child: Container(
                    decoration: BoxDecoration(
                      color: Colors.black54,
                      borderRadius: BorderRadius.circular(2.0),
                    ),
                    padding: const EdgeInsets.all(4.0),
                    child: RichText(
                      text: const TextSpan(
                        style: TextStyle(color: Colors.white),
                        children: <TextSpan>[
                          TextSpan(text: 'Photo by '),
                          TextSpan(
                            style: TextStyle(fontWeight: FontWeight.bold),
                            text: 'Chris Godley',
                          ),
                        ],
                      ),
                    ),
                  ),
                ),
              ],
            ),
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  Text('Artisans of Southern India', style: Theme.of(context).textTheme.bodyLarge),
                  Text('Silk Spinners', style: Theme.of(context).textTheme.bodyMedium),
                  Text('Sivaganga, Tamil Nadu', style: Theme.of(context).textTheme.bodySmall),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class ItemGalleryBox extends StatelessWidget {
  const ItemGalleryBox(this.index, {super.key});

  final int index;

  @override
  Widget build(BuildContext context) {
    final List<String> tabNames = <String>['A', 'B', 'C', 'D'];

    return SizedBox(
      height: 200.0,
      child: DefaultTabController(
        length: tabNames.length,
        child: Column(
          children: <Widget>[
            Expanded(
              child: TabBarView(
                children:
                    tabNames.map<Widget>((String tabName) {
                      return Container(
                        key: PageStorageKey<String>(tabName),
                        child: Padding(
                          padding: const EdgeInsets.all(8.0),
                          child: Card(
                            child: Column(
                              children: <Widget>[
                                Expanded(
                                  child: ColoredBox(
                                    color: Theme.of(context).primaryColor,
                                    child: Center(
                                      child: Text(
                                        tabName,
                                        style: Theme.of(
                                          context,
                                        ).textTheme.headlineSmall!.copyWith(color: Colors.white),
                                      ),
                                    ),
                                  ),
                                ),
                                Row(
                                  children: <Widget>[
                                    IconButton(
                                      icon: const Icon(Icons.share),
                                      onPressed: () {
                                        print('Pressed share');
                                      },
                                    ),
                                    IconButton(
                                      icon: const Icon(Icons.event),
                                      onPressed: () {
                                        print('Pressed event');
                                      },
                                    ),
                                    Expanded(
                                      child: Padding(
                                        padding: const EdgeInsets.only(left: 8.0),
                                        child: Text('This is item $tabName'),
                                      ),
                                    ),
                                  ],
                                ),
                              ],
                            ),
                          ),
                        ),
                      );
                    }).toList(),
              ),
            ),
            const TabPageSelector(),
          ],
        ),
      ),
    );
  }
}

class BottomBar extends StatelessWidget {
  const BottomBar({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        border: Border(top: BorderSide(color: Theme.of(context).dividerColor)),
      ),
      child: const Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: <Widget>[
          BottomBarButton(Icons.new_releases, 'News'),
          BottomBarButton(Icons.people, 'Requests'),
          BottomBarButton(Icons.chat, 'Messenger'),
          BottomBarButton(Icons.bookmark, 'Bookmark'),
          BottomBarButton(Icons.alarm, 'Alarm'),
        ],
      ),
    );
  }
}

class BottomBarButton extends StatelessWidget {
  const BottomBarButton(this.icon, this.title, {super.key});

  final IconData icon;
  final String title;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Column(
        children: <Widget>[
          IconButton(
            icon: Icon(icon),
            onPressed: () {
              print('Pressed: $title');
            },
          ),
          Text(title, style: Theme.of(context).textTheme.bodySmall),
        ],
      ),
    );
  }
}

class GalleryDrawer extends StatelessWidget {
  const GalleryDrawer({super.key});

  void _changeTheme(BuildContext context, bool value) {
    ComplexLayoutApp.of(context)?.lightTheme = value;
  }

  void _changeScrollMode(BuildContext context, ScrollMode mode) {
    ComplexLayoutApp.of(context)?.scrollMode = mode;
  }

  @override
  Widget build(BuildContext context) {
    final ScrollMode currentMode = ComplexLayoutApp.of(context)!.scrollMode;
    return Drawer(
      // For real apps, see the Gallery material Drawer demo. More
      // typically, a drawer would have a fixed header with a scrolling body
      // below it.
      child: ListView(
        key: const PageStorageKey<String>('gallery-drawer'),
        padding: EdgeInsets.zero,
        children: <Widget>[
          const FancyDrawerHeader(),
          ListTile(
            key: const Key('scroll-switcher'),
            title: const Text('Scroll Mode'),
            onTap: () {
              _changeScrollMode(
                context,
                currentMode == ScrollMode.complex ? ScrollMode.tile : ScrollMode.complex,
              );
              Navigator.pop(context);
            },
            trailing: Text(currentMode == ScrollMode.complex ? 'Tile' : 'Complex'),
          ),
          ListTile(
            leading: const Icon(Icons.brightness_5),
            title: const Text('Light'),
            onTap: () {
              _changeTheme(context, true);
            },
            selected: ComplexLayoutApp.of(context)!.lightTheme,
            trailing: Radio<bool>(
              value: true,
              groupValue: ComplexLayoutApp.of(context)!.lightTheme,
              onChanged: (bool? value) {
                _changeTheme(context, value!);
              },
            ),
          ),
          ListTile(
            leading: const Icon(Icons.brightness_7),
            title: const Text('Dark'),
            onTap: () {
              _changeTheme(context, false);
            },
            selected: !ComplexLayoutApp.of(context)!.lightTheme,
            trailing: Radio<bool>(
              value: false,
              groupValue: ComplexLayoutApp.of(context)!.lightTheme,
              onChanged: (bool? value) {
                _changeTheme(context, value!);
              },
            ),
          ),
          const Divider(),
          ListTile(
            leading: const Icon(Icons.hourglass_empty),
            title: const Text('Animate Slowly'),
            selected: timeDilation != 1.0,
            onTap: () {
              ComplexLayoutApp.of(context)!.toggleAnimationSpeed();
            },
            trailing: Checkbox(
              value: timeDilation != 1.0,
              onChanged: (bool? value) {
                ComplexLayoutApp.of(context)!.toggleAnimationSpeed();
              },
            ),
          ),
        ],
      ),
    );
  }
}

class FancyDrawerHeader extends StatelessWidget {
  const FancyDrawerHeader({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      color: Colors.purple,
      height: 200.0,
      child: const SafeArea(bottom: false, child: Placeholder()),
    );
  }
}
