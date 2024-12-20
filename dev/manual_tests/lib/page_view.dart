// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class CardModel {
  CardModel(this.value, this.size, this.color);
  int value;
  Size size;
  Color color;
  String get label => 'Card $value';
  Key get key => ObjectKey(this);
}

class PageViewApp extends StatefulWidget {
  const PageViewApp({super.key});

  @override
  PageViewAppState createState() => PageViewAppState();
}

class PageViewAppState extends State<PageViewApp> {
  @override
  void initState() {
    super.initState();
    const List<Size> cardSizes = <Size>[
      Size(100.0, 300.0),
      Size(300.0, 100.0),
      Size(200.0, 400.0),
      Size(400.0, 400.0),
      Size(300.0, 400.0),
    ];

    cardModels = List<CardModel>.generate(cardSizes.length, (int i) {
      final Color? color = Color.lerp(
        Colors.red.shade300,
        Colors.blue.shade900,
        i / cardSizes.length,
      );
      return CardModel(i, cardSizes[i], color!);
    });
  }

  static const TextStyle cardLabelStyle = TextStyle(
    color: Colors.white,
    fontSize: 18.0,
    fontWeight: FontWeight.bold,
  );

  List<CardModel> cardModels = <CardModel>[];
  Size pageSize = const Size(200.0, 200.0);
  Axis scrollDirection = Axis.horizontal;
  bool itemsWrap = false;

  Widget buildCard(CardModel cardModel) {
    final Widget card = Card(
      color: cardModel.color,
      child: Container(
        width: cardModel.size.width,
        height: cardModel.size.height,
        padding: const EdgeInsets.all(8.0),
        child: Center(child: Text(cardModel.label, style: cardLabelStyle)),
      ),
    );

    final BoxConstraints constraints =
        (scrollDirection == Axis.vertical)
            ? BoxConstraints.tightFor(height: pageSize.height)
            : BoxConstraints.tightFor(width: pageSize.width);

    return Container(key: cardModel.key, constraints: constraints, child: Center(child: card));
  }

  void switchScrollDirection() {
    setState(() {
      scrollDirection = flipAxis(scrollDirection);
    });
  }

  void toggleItemsWrap() {
    setState(() {
      itemsWrap = !itemsWrap;
    });
  }

  Widget _buildDrawer() {
    return Drawer(
      child: ListView(
        children: <Widget>[
          const DrawerHeader(child: Center(child: Text('Options'))),
          ListTile(
            leading: const Icon(Icons.more_horiz),
            selected: scrollDirection == Axis.horizontal,
            trailing: const Text('Horizontal Layout'),
            onTap: switchScrollDirection,
          ),
          ListTile(
            leading: const Icon(Icons.more_vert),
            selected: scrollDirection == Axis.vertical,
            trailing: const Text('Vertical Layout'),
            onTap: switchScrollDirection,
          ),
          ListTile(
            onTap: toggleItemsWrap,
            title: const Text('Scrolling wraps around'),
            // TODO(abarth): Actually make this checkbox change this value.
            trailing: Checkbox(value: itemsWrap, onChanged: null),
          ),
        ],
      ),
    );
  }

  AppBar _buildAppBar() {
    return AppBar(title: const Text('PageView'), actions: <Widget>[Text(scrollDirection.name)]);
  }

  Widget _buildBody(BuildContext context) {
    return PageView(
      // TODO(abarth): itemsWrap: itemsWrap,
      scrollDirection: scrollDirection,
      children: cardModels.map<Widget>(buildCard).toList(),
    );
  }

  @override
  Widget build(BuildContext context) {
    return IconTheme(
      data: const IconThemeData(color: Colors.white),
      child: Scaffold(appBar: _buildAppBar(), drawer: _buildDrawer(), body: _buildBody(context)),
    );
  }
}

void main() {
  runApp(const MaterialApp(title: 'PageView', home: PageViewApp()));
}
