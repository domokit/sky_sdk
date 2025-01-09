// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

const String _kGalleryAssetsPackage = 'flutter_gallery_assets';

const List<Color> coolColors = <Color>[
  Color.fromARGB(255, 255, 59, 48),
  Color.fromARGB(255, 255, 149, 0),
  Color.fromARGB(255, 255, 204, 0),
  Color.fromARGB(255, 76, 217, 100),
  Color.fromARGB(255, 90, 200, 250),
  Color.fromARGB(255, 0, 122, 255),
  Color.fromARGB(255, 88, 86, 214),
  Color.fromARGB(255, 255, 45, 85),
];

const List<String> coolColorNames = <String>[
  'Sarcoline',
  'Coquelicot',
  'Smaragdine',
  'Mikado',
  'Glaucous',
  'Wenge',
  'Fulvous',
  'Xanadu',
  'Falu',
  'Eburnean',
  'Amaranth',
  'Australien',
  'Banan',
  'Falu',
  'Gingerline',
  'Incarnadine',
  'Labrador',
  'Nattier',
  'Pervenche',
  'Sinoper',
  'Verditer',
  'Watchet',
  'Zaffre',
];

const int _kChildCount = 50;

class CupertinoNavigationDemo extends StatelessWidget {
  CupertinoNavigationDemo({super.key, this.randomSeed})
    : colorItems = List<Color>.generate(_kChildCount, (int index) {
        return coolColors[math.Random(randomSeed).nextInt(coolColors.length)];
      }),
      colorNameItems = List<String>.generate(_kChildCount, (int index) {
        return coolColorNames[math.Random(randomSeed).nextInt(coolColorNames.length)];
      });

  static const String routeName = '/cupertino/navigation';

  final List<Color> colorItems;
  final List<String> colorNameItems;
  final int? randomSeed;

  @override
  Widget build(BuildContext context) {
    return PopScope<Object?>(
      // Prevent swipe popping of this page. Use explicit exit buttons only.
      canPop: false,
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle,
        child: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(CupertinoIcons.house, size: 27), label: 'Home'),
              BottomNavigationBarItem(
                icon: Icon(CupertinoIcons.chat_bubble, size: 27),
                label: 'Support',
              ),
              BottomNavigationBarItem(
                icon: Icon(CupertinoIcons.person_circle, size: 27),
                label: 'Profile',
              ),
            ],
          ),
          tabBuilder: (BuildContext context, int index) {
            switch (index) {
              case 0:
                return CupertinoTabView(
                  builder: (BuildContext context) {
                    return CupertinoDemoTab1(
                      colorItems: colorItems,
                      colorNameItems: colorNameItems,
                      randomSeed: randomSeed,
                    );
                  },
                  defaultTitle: 'Colors',
                );
              case 1:
                return CupertinoTabView(
                  builder: (BuildContext context) => const CupertinoDemoTab2(),
                  defaultTitle: 'Support Chat',
                );
              case 2:
                return CupertinoTabView(
                  builder: (BuildContext context) => const CupertinoDemoTab3(),
                  defaultTitle: 'Account',
                );
            }
            assert(false);
            return const CupertinoTabView();
          },
        ),
      ),
    );
  }
}

class ExitButton extends StatelessWidget {
  const ExitButton({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoButton(
      padding: EdgeInsets.zero,
      child: const Tooltip(message: 'Back', excludeFromSemantics: true, child: Text('Exit')),
      onPressed: () {
        // The demo is on the root navigator.
        Navigator.of(context, rootNavigator: true).pop();
      },
    );
  }
}

final Widget trailingButtons = Row(
  mainAxisSize: MainAxisSize.min,
  children: <Widget>[
    CupertinoDemoDocumentationButton(CupertinoNavigationDemo.routeName),
    const Padding(padding: EdgeInsets.only(left: 8.0)),
    const ExitButton(),
  ],
);

class CupertinoDemoTab1 extends StatelessWidget {
  const CupertinoDemoTab1({super.key, this.colorItems, this.colorNameItems, this.randomSeed});

  final List<Color>? colorItems;
  final List<String>? colorNameItems;
  final int? randomSeed;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      backgroundColor: CupertinoColors.systemGroupedBackground,
      child: CustomScrollView(
        semanticChildCount: _kChildCount,
        slivers: <Widget>[
          CupertinoSliverNavigationBar(trailing: trailingButtons),
          SliverPadding(
            // Top media padding consumed by CupertinoSliverNavigationBar.
            // Left/Right media padding consumed by Tab1RowItem.
            padding:
                MediaQuery.of(
                  context,
                ).removePadding(removeTop: true, removeLeft: true, removeRight: true).padding,
            sliver: SliverList(
              delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                return Tab1RowItem(
                  index: index,
                  lastItem: index == _kChildCount - 1,
                  color: colorItems![index],
                  colorName: colorNameItems![index],
                  randomSeed: randomSeed,
                );
              }, childCount: _kChildCount),
            ),
          ),
        ],
      ),
    );
  }
}

class Tab1RowItem extends StatelessWidget {
  const Tab1RowItem({
    super.key,
    this.index,
    this.lastItem,
    this.color,
    this.colorName,
    this.randomSeed,
  });

  final int? index;
  final bool? lastItem;
  final Color? color;
  final String? colorName;
  final int? randomSeed;

  @override
  Widget build(BuildContext context) {
    final Widget row = GestureDetector(
      behavior: HitTestBehavior.opaque,
      onTap: () {
        Navigator.of(context).push(
          CupertinoPageRoute<void>(
            title: colorName,
            builder:
                (BuildContext context) => Tab1ItemPage(
                  color: color,
                  colorName: colorName,
                  index: index,
                  randomSeed: randomSeed,
                ),
          ),
        );
      },
      child: ColoredBox(
        color: CupertinoDynamicColor.resolve(CupertinoColors.systemBackground, context),
        child: SafeArea(
          top: false,
          bottom: false,
          child: Padding(
            padding: const EdgeInsets.only(left: 16.0, top: 8.0, bottom: 8.0, right: 8.0),
            child: Row(
              children: <Widget>[
                Container(
                  height: 60.0,
                  width: 60.0,
                  decoration: BoxDecoration(color: color, borderRadius: BorderRadius.circular(8.0)),
                ),
                Expanded(
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 12.0),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: <Widget>[
                        Text(colorName!),
                        const Padding(padding: EdgeInsets.only(top: 8.0)),
                        Text(
                          'Buy this cool color',
                          style: TextStyle(
                            color: CupertinoDynamicColor.resolve(
                              CupertinoColors.secondaryLabel,
                              context,
                            ),
                            fontSize: 13.0,
                            fontWeight: FontWeight.w300,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
                CupertinoButton(
                  padding: EdgeInsets.zero,
                  child: const Icon(CupertinoIcons.plus_circled, semanticLabel: 'Add'),
                  onPressed: () {},
                ),
                CupertinoButton(
                  padding: EdgeInsets.zero,
                  child: const Icon(CupertinoIcons.share, semanticLabel: 'Share'),
                  onPressed: () {},
                ),
              ],
            ),
          ),
        ),
      ),
    );

    if (lastItem!) {
      return row;
    }

    return Column(
      children: <Widget>[
        row,
        Container(
          height: 1.0,
          color: CupertinoDynamicColor.resolve(CupertinoColors.separator, context),
        ),
      ],
    );
  }
}

class Tab1ItemPage extends StatefulWidget {
  const Tab1ItemPage({super.key, this.color, this.colorName, this.index, this.randomSeed});

  final Color? color;
  final String? colorName;
  final int? index;
  final int? randomSeed;

  @override
  State<StatefulWidget> createState() => Tab1ItemPageState();
}

class Tab1ItemPageState extends State<Tab1ItemPage> {
  late final List<Color> relatedColors = List<Color>.generate(10, (int index) {
    final math.Random random = math.Random(widget.randomSeed);
    return Color.fromARGB(
      255,
      (widget.color!.red + random.nextInt(100) - 50).clamp(0, 255),
      (widget.color!.green + random.nextInt(100) - 50).clamp(0, 255),
      (widget.color!.blue + random.nextInt(100) - 50).clamp(0, 255),
    );
  });

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(trailing: ExitButton()),
      child: SafeArea(
        top: false,
        bottom: false,
        child: ListView(
          children: <Widget>[
            const Padding(padding: EdgeInsets.only(top: 16.0)),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16.0),
              child: Row(
                children: <Widget>[
                  Container(
                    height: 128.0,
                    width: 128.0,
                    decoration: BoxDecoration(
                      color: widget.color,
                      borderRadius: BorderRadius.circular(24.0),
                    ),
                  ),
                  const Padding(padding: EdgeInsets.only(left: 18.0)),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      mainAxisSize: MainAxisSize.min,
                      children: <Widget>[
                        Text(
                          widget.colorName!,
                          style: const TextStyle(fontSize: 24.0, fontWeight: FontWeight.bold),
                        ),
                        const Padding(padding: EdgeInsets.only(top: 6.0)),
                        Text(
                          'Item number ${widget.index}',
                          style: TextStyle(
                            color: CupertinoDynamicColor.resolve(
                              CupertinoColors.secondaryLabel,
                              context,
                            ),
                            fontSize: 16.0,
                            fontWeight: FontWeight.w100,
                          ),
                        ),
                        const Padding(padding: EdgeInsets.only(top: 20.0)),
                        Row(
                          mainAxisAlignment: MainAxisAlignment.spaceBetween,
                          children: <Widget>[
                            CupertinoButton.filled(
                              minSize: 30.0,
                              padding: const EdgeInsets.symmetric(horizontal: 24.0),
                              borderRadius: BorderRadius.circular(32.0),
                              child: const Text(
                                'GET',
                                style: TextStyle(
                                  fontSize: 14.0,
                                  fontWeight: FontWeight.w700,
                                  letterSpacing: -0.28,
                                ),
                              ),
                              onPressed: () {},
                            ),
                            CupertinoButton.filled(
                              minSize: 30.0,
                              padding: EdgeInsets.zero,
                              borderRadius: BorderRadius.circular(32.0),
                              child: const Icon(CupertinoIcons.ellipsis),
                              onPressed: () {},
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
            const Padding(
              padding: EdgeInsets.only(left: 16.0, top: 28.0, bottom: 8.0),
              child: Text(
                'USERS ALSO LIKED',
                style: TextStyle(
                  color: Color(0xFF646464),
                  letterSpacing: -0.60,
                  fontSize: 15.0,
                  fontWeight: FontWeight.w500,
                ),
              ),
            ),
            SizedBox(
              height: 200.0,
              child: ListView.builder(
                scrollDirection: Axis.horizontal,
                itemCount: 10,
                itemExtent: 160.0,
                itemBuilder: (BuildContext context, int index) {
                  return Padding(
                    padding: const EdgeInsets.only(left: 16.0),
                    child: Container(
                      decoration: BoxDecoration(
                        borderRadius: BorderRadius.circular(8.0),
                        color: relatedColors[index],
                      ),
                      child: Center(
                        child: CupertinoButton(
                          child: const Icon(
                            CupertinoIcons.plus_circled,
                            color: CupertinoColors.white,
                            size: 36.0,
                          ),
                          onPressed: () {},
                        ),
                      ),
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class CupertinoDemoTab2 extends StatelessWidget {
  const CupertinoDemoTab2({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(trailing: trailingButtons),
      child: CupertinoScrollbar(
        child: ListView(
          primary: true,
          children: <Widget>[
            const CupertinoUserInterfaceLevel(
              data: CupertinoUserInterfaceLevelData.elevated,
              child: Tab2Header(),
            ),
            ...buildTab2Conversation(),
          ],
        ),
      ),
    );
  }
}

class Tab2Header extends StatelessWidget {
  const Tab2Header({super.key});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16.0),
      child: SafeArea(
        top: false,
        bottom: false,
        child: ClipRRect(
          borderRadius: const BorderRadius.all(Radius.circular(16.0)),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Container(
                decoration: BoxDecoration(
                  color: CupertinoDynamicColor.resolve(CupertinoColors.systemFill, context),
                ),
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 18.0, vertical: 12.0),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: <Widget>[
                      Text(
                        'SUPPORT TICKET',
                        style: TextStyle(
                          color: CupertinoDynamicColor.resolve(
                            CupertinoColors.secondaryLabel,
                            context,
                          ),
                          letterSpacing: -0.9,
                          fontSize: 14.0,
                          fontWeight: FontWeight.w500,
                        ),
                      ),
                      Text(
                        'Show More',
                        style: TextStyle(
                          color: CupertinoDynamicColor.resolve(
                            CupertinoColors.secondaryLabel,
                            context,
                          ),
                          letterSpacing: -0.6,
                          fontSize: 12.0,
                          fontWeight: FontWeight.w500,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
              Container(
                decoration: BoxDecoration(
                  color: CupertinoDynamicColor.resolve(
                    CupertinoColors.quaternarySystemFill,
                    context,
                  ),
                ),
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 18.0, vertical: 12.0),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      const Text(
                        'Product or product packaging damaged during transit',
                        style: TextStyle(
                          fontSize: 16.0,
                          fontWeight: FontWeight.w700,
                          letterSpacing: -0.46,
                        ),
                      ),
                      const Padding(padding: EdgeInsets.only(top: 16.0)),
                      const Text(
                        'REVIEWERS',
                        style: TextStyle(
                          color: Color(0xFF646464),
                          fontSize: 12.0,
                          letterSpacing: -0.6,
                          fontWeight: FontWeight.w500,
                        ),
                      ),
                      const Padding(padding: EdgeInsets.only(top: 8.0)),
                      Row(
                        children: <Widget>[
                          Container(
                            width: 44.0,
                            height: 44.0,
                            decoration: const BoxDecoration(
                              image: DecorationImage(
                                image: AssetImage(
                                  'people/square/trevor.png',
                                  package: _kGalleryAssetsPackage,
                                ),
                              ),
                              shape: BoxShape.circle,
                            ),
                          ),
                          const Padding(padding: EdgeInsets.only(left: 8.0)),
                          Container(
                            width: 44.0,
                            height: 44.0,
                            decoration: const BoxDecoration(
                              image: DecorationImage(
                                image: AssetImage(
                                  'people/square/sandra.png',
                                  package: _kGalleryAssetsPackage,
                                ),
                              ),
                              shape: BoxShape.circle,
                            ),
                          ),
                          const Padding(padding: EdgeInsets.only(left: 2.0)),
                          const Icon(
                            CupertinoIcons.check_mark_circled,
                            color: Color(0xFF646464),
                            size: 20.0,
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
      ),
    );
  }
}

enum Tab2ConversationBubbleColor { blue, gray }

class Tab2ConversationBubble extends StatelessWidget {
  const Tab2ConversationBubble({super.key, this.text, this.color});

  final String? text;
  final Tab2ConversationBubbleColor? color;

  @override
  Widget build(BuildContext context) {
    Color? backgroundColor;
    Color? foregroundColor;

    switch (color) {
      case Tab2ConversationBubbleColor.gray:
        backgroundColor = CupertinoDynamicColor.resolve(CupertinoColors.systemFill, context);
        foregroundColor = CupertinoDynamicColor.resolve(CupertinoColors.label, context);
      case Tab2ConversationBubbleColor.blue:
        backgroundColor = CupertinoTheme.of(context).primaryColor;
        foregroundColor = CupertinoColors.white;
      case null:
        break;
    }

    return Container(
      decoration: BoxDecoration(
        borderRadius: const BorderRadius.all(Radius.circular(18.0)),
        color: backgroundColor,
      ),
      margin: const EdgeInsets.symmetric(horizontal: 8.0, vertical: 8.0),
      padding: const EdgeInsets.symmetric(horizontal: 14.0, vertical: 10.0),
      child: Text(
        text!,
        style: TextStyle(
          color: foregroundColor,
          letterSpacing: -0.4,
          fontSize: 15.0,
          fontWeight: FontWeight.w400,
        ),
      ),
    );
  }
}

class Tab2ConversationAvatar extends StatelessWidget {
  const Tab2ConversationAvatar({super.key, this.text, this.color});

  final String? text;
  final Color? color;

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        gradient: LinearGradient(
          begin: FractionalOffset.topCenter,
          end: FractionalOffset.bottomCenter,
          colors: <Color>[
            color!,
            Color.fromARGB(
              color!.alpha,
              (color!.red - 60).clamp(0, 255),
              (color!.green - 60).clamp(0, 255),
              (color!.blue - 60).clamp(0, 255),
            ),
          ],
        ),
      ),
      margin: const EdgeInsets.only(left: 8.0, bottom: 8.0),
      padding: const EdgeInsets.all(12.0),
      child: Text(
        text!,
        style: const TextStyle(
          color: CupertinoColors.white,
          fontSize: 13.0,
          fontWeight: FontWeight.w500,
        ),
      ),
    );
  }
}

class Tab2ConversationRow extends StatelessWidget {
  const Tab2ConversationRow({super.key, this.avatar, this.text});

  final Tab2ConversationAvatar? avatar;
  final String? text;

  @override
  Widget build(BuildContext context) {
    final bool isSelf = avatar == null;
    return SafeArea(
      child: Row(
        mainAxisAlignment: isSelf ? MainAxisAlignment.end : MainAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: isSelf ? CrossAxisAlignment.center : CrossAxisAlignment.end,
        children: <Widget>[
          if (avatar != null) avatar!,
          CupertinoUserInterfaceLevel(
            data: CupertinoUserInterfaceLevelData.elevated,
            child: Tab2ConversationBubble(
              text: text,
              color: isSelf ? Tab2ConversationBubbleColor.blue : Tab2ConversationBubbleColor.gray,
            ),
          ),
        ],
      ),
    );
  }
}

List<Widget> buildTab2Conversation() {
  return <Widget>[
    const Tab2ConversationRow(text: "My Xanadu doesn't look right"),
    const Tab2ConversationRow(
      avatar: Tab2ConversationAvatar(text: 'KL', color: Color(0xFFFD5015)),
      text: "We'll rush you a new one.\nIt's gonna be incredible",
    ),
    const Tab2ConversationRow(text: 'Awesome thanks!'),
    const Tab2ConversationRow(
      avatar: Tab2ConversationAvatar(text: 'SJ', color: Color(0xFF34CAD6)),
      text: "We'll send you our\nnewest Labrador too!",
    ),
    const Tab2ConversationRow(text: 'Yay'),
    const Tab2ConversationRow(
      avatar: Tab2ConversationAvatar(text: 'KL', color: Color(0xFFFD5015)),
      text: "Actually there's one more thing...",
    ),
    const Tab2ConversationRow(text: "What's that?"),
  ];
}

class CupertinoDemoTab3 extends StatelessWidget {
  const CupertinoDemoTab3({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(trailing: trailingButtons),
      backgroundColor: CupertinoColors.systemBackground,
      child: ListView(
        children: <Widget>[
          const Padding(padding: EdgeInsets.only(top: 32.0)),
          GestureDetector(
            onTap: () {
              Navigator.of(context, rootNavigator: true).push(
                CupertinoPageRoute<bool>(
                  fullscreenDialog: true,
                  builder: (BuildContext context) => const Tab3Dialog(),
                ),
              );
            },
            child: Container(
              decoration: BoxDecoration(
                color: CupertinoTheme.of(context).scaffoldBackgroundColor,
                border: const Border(
                  top: BorderSide(color: Color(0xFFBCBBC1), width: 0.0),
                  bottom: BorderSide(color: Color(0xFFBCBBC1), width: 0.0),
                ),
              ),
              height: 44.0,
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
                child: SafeArea(
                  top: false,
                  bottom: false,
                  child: Row(
                    children: <Widget>[
                      Text(
                        'Sign in',
                        style: TextStyle(color: CupertinoTheme.of(context).primaryColor),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class Tab3Dialog extends StatelessWidget {
  const Tab3Dialog({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        leading: CupertinoButton(
          padding: EdgeInsets.zero,
          child: const Text('Cancel'),
          onPressed: () {
            Navigator.of(context).pop(false);
          },
        ),
      ),
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            const Icon(CupertinoIcons.profile_circled, size: 160.0, color: Color(0xFF646464)),
            const Padding(padding: EdgeInsets.only(top: 18.0)),
            CupertinoButton.filled(
              child: const Text('Sign in'),
              onPressed: () {
                Navigator.pop(context);
              },
            ),
          ],
        ),
      ),
    );
  }
}
