// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/material.dart';

class CardModel {
  CardModel(this.value, this.height) : textController = TextEditingController(text: 'Item $value');

  int value;
  double height;
  int get color => ((value % 9) + 1) * 100;
  final TextEditingController textController;
  Key get key => ObjectKey(this);
}

class CardCollection extends StatefulWidget {
  const CardCollection({super.key});

  @override
  CardCollectionState createState() => CardCollectionState();
}

class CardCollectionState extends State<CardCollection> {
  static const TextStyle cardLabelStyle = TextStyle(
    color: Colors.white,
    fontSize: 18.0,
    fontWeight: FontWeight.bold,
  );

  // TODO(hansmuller): need a local image asset
  static const String _sunshineURL =
      'http://www.walltor.com/images/wallpaper/good-morning-sunshine-58540.jpg';

  static const double kCardMargins = 8.0;
  static const double kFixedCardHeight = 100.0;
  static const List<double> _cardHeights = <double>[
    48.0,
    63.0,
    85.0,
    146.0,
    60.0,
    55.0,
    84.0,
    96.0,
    50.0,
    48.0,
    63.0,
    85.0,
    146.0,
    60.0,
    55.0,
    84.0,
    96.0,
    50.0,
    48.0,
    63.0,
    85.0,
    146.0,
    60.0,
    55.0,
    84.0,
    96.0,
    50.0,
  ];

  MaterialColor _primaryColor = Colors.deepPurple;
  List<CardModel> _cardModels = <CardModel>[];
  DismissDirection _dismissDirection = DismissDirection.horizontal;
  TextAlign _textAlign = TextAlign.center;
  bool _editable = false;
  bool _fixedSizeCards = false;
  bool _sunshine = false;
  bool _varyFontSizes = false;

  void _updateCardSizes() {
    if (_fixedSizeCards) {
      return;
    }
    _cardModels = List<CardModel>.generate(_cardModels.length, (int i) {
      _cardModels[i].height = _editable ? max(_cardHeights[i], 60.0) : _cardHeights[i];
      return _cardModels[i];
    });
  }

  void _initVariableSizedCardModels() {
    _cardModels = List<CardModel>.generate(
      _cardHeights.length,
      (int i) => CardModel(i, _editable ? max(_cardHeights[i], 60.0) : _cardHeights[i]),
    );
  }

  void _initFixedSizedCardModels() {
    const int cardCount = 27;
    _cardModels = List<CardModel>.generate(cardCount, (int i) => CardModel(i, kFixedCardHeight));
  }

  void _initCardModels() {
    if (_fixedSizeCards) {
      _initFixedSizedCardModels();
    } else {
      _initVariableSizedCardModels();
    }
  }

  @override
  void initState() {
    super.initState();
    _initCardModels();
  }

  void dismissCard(CardModel card) {
    if (_cardModels.contains(card)) {
      setState(() {
        _cardModels.remove(card);
      });
    }
  }

  Widget _buildDrawer() {
    return Drawer(
      child: IconTheme(
        data: const IconThemeData(color: Colors.black),
        child: ListView(
          children: <Widget>[
            const DrawerHeader(child: Center(child: Text('Options'))),
            buildDrawerCheckbox('Make card labels editable', _editable, _toggleEditable),
            buildDrawerCheckbox('Fixed size cards', _fixedSizeCards, _toggleFixedSizeCards),
            buildDrawerCheckbox('Let the sun shine', _sunshine, _toggleSunshine),
            buildDrawerCheckbox(
              'Vary font sizes',
              _varyFontSizes,
              _toggleVaryFontSizes,
              enabled: !_editable,
            ),
            const Divider(),
            buildDrawerColorRadioItem(
              'Deep Purple',
              Colors.deepPurple,
              _primaryColor,
              _selectColor,
            ),
            buildDrawerColorRadioItem('Green', Colors.green, _primaryColor, _selectColor),
            buildDrawerColorRadioItem('Amber', Colors.amber, _primaryColor, _selectColor),
            buildDrawerColorRadioItem('Teal', Colors.teal, _primaryColor, _selectColor),
            const Divider(),
            buildDrawerDirectionRadioItem(
              'Dismiss horizontally',
              DismissDirection.horizontal,
              _dismissDirection,
              _changeDismissDirection,
              icon: Icons.code,
            ),
            buildDrawerDirectionRadioItem(
              'Dismiss left',
              DismissDirection.endToStart,
              _dismissDirection,
              _changeDismissDirection,
              icon: Icons.arrow_back,
            ),
            buildDrawerDirectionRadioItem(
              'Dismiss right',
              DismissDirection.startToEnd,
              _dismissDirection,
              _changeDismissDirection,
              icon: Icons.arrow_forward,
            ),
            const Divider(),
            buildFontRadioItem(
              'Left-align text',
              TextAlign.left,
              _textAlign,
              _changeTextAlign,
              icon: Icons.format_align_left,
              enabled: !_editable,
            ),
            buildFontRadioItem(
              'Center-align text',
              TextAlign.center,
              _textAlign,
              _changeTextAlign,
              icon: Icons.format_align_center,
              enabled: !_editable,
            ),
            buildFontRadioItem(
              'Right-align text',
              TextAlign.right,
              _textAlign,
              _changeTextAlign,
              icon: Icons.format_align_right,
              enabled: !_editable,
            ),
            const Divider(),
            ListTile(
              leading: const Icon(Icons.dvr),
              onTap: () {
                debugDumpApp();
                debugDumpRenderTree();
              },
              title: const Text('Dump App to Console'),
            ),
          ],
        ),
      ),
    );
  }

  String _dismissDirectionText(DismissDirection direction) {
    final String s = direction.toString();
    return "dismiss ${s.substring(s.indexOf('.') + 1)}";
  }

  void _toggleEditable() {
    setState(() {
      _editable = !_editable;
      _updateCardSizes();
    });
  }

  void _toggleFixedSizeCards() {
    setState(() {
      _fixedSizeCards = !_fixedSizeCards;
      _initCardModels();
    });
  }

  void _toggleSunshine() {
    setState(() {
      _sunshine = !_sunshine;
    });
  }

  void _toggleVaryFontSizes() {
    setState(() {
      _varyFontSizes = !_varyFontSizes;
    });
  }

  void _selectColor(MaterialColor? selection) {
    setState(() {
      _primaryColor = selection!;
    });
  }

  void _changeDismissDirection(DismissDirection? newDismissDirection) {
    setState(() {
      _dismissDirection = newDismissDirection!;
    });
  }

  void _changeTextAlign(TextAlign? newTextAlign) {
    setState(() {
      _textAlign = newTextAlign!;
    });
  }

  Widget buildDrawerCheckbox(
    String label,
    bool value,
    void Function() callback, {
    bool enabled = true,
  }) {
    return ListTile(
      onTap: enabled ? callback : null,
      title: Text(label),
      trailing: Checkbox(
        value: value,
        onChanged:
            enabled
                ? (_) {
                  callback();
                }
                : null,
      ),
    );
  }

  Widget buildDrawerColorRadioItem(
    String label,
    MaterialColor itemValue,
    MaterialColor currentValue,
    ValueChanged<MaterialColor?> onChanged, {
    IconData? icon,
    bool enabled = true,
  }) {
    return ListTile(
      leading: Icon(icon),
      title: Text(label),
      onTap:
          enabled
              ? () {
                onChanged(itemValue);
              }
              : null,
      trailing: Radio<MaterialColor>(
        value: itemValue,
        groupValue: currentValue,
        onChanged: enabled ? onChanged : null,
      ),
    );
  }

  Widget buildDrawerDirectionRadioItem(
    String label,
    DismissDirection itemValue,
    DismissDirection currentValue,
    ValueChanged<DismissDirection?> onChanged, {
    IconData? icon,
    bool enabled = true,
  }) {
    return ListTile(
      leading: Icon(icon),
      title: Text(label),
      onTap:
          enabled
              ? () {
                onChanged(itemValue);
              }
              : null,
      trailing: Radio<DismissDirection>(
        value: itemValue,
        groupValue: currentValue,
        onChanged: enabled ? onChanged : null,
      ),
    );
  }

  Widget buildFontRadioItem(
    String label,
    TextAlign itemValue,
    TextAlign currentValue,
    ValueChanged<TextAlign?> onChanged, {
    IconData? icon,
    bool enabled = true,
  }) {
    return ListTile(
      leading: Icon(icon),
      title: Text(label),
      onTap:
          enabled
              ? () {
                onChanged(itemValue);
              }
              : null,
      trailing: Radio<TextAlign>(
        value: itemValue,
        groupValue: currentValue,
        onChanged: enabled ? onChanged : null,
      ),
    );
  }

  AppBar _buildAppBar(BuildContext context) {
    return AppBar(
      actions: <Widget>[Text(_dismissDirectionText(_dismissDirection))],
      flexibleSpace: Container(
        padding: const EdgeInsets.only(left: 72.0),
        height: 128.0,
        alignment: const Alignment(-1.0, 0.5),
        child: Text(
          'Swipe Away: ${_cardModels.length}',
          style: Theme.of(context).primaryTextTheme.titleLarge,
        ),
      ),
    );
  }

  Widget _buildCard(BuildContext context, int index) {
    final CardModel cardModel = _cardModels[index];
    final Widget card = Dismissible(
      key: ObjectKey(cardModel),
      direction: _dismissDirection,
      onDismissed: (DismissDirection direction) {
        dismissCard(cardModel);
      },
      child: Card(
        color: _primaryColor[cardModel.color],
        child: Container(
          height: cardModel.height,
          padding: const EdgeInsets.all(kCardMargins),
          child:
              _editable
                  ? Center(
                    child: TextField(
                      key: GlobalObjectKey(cardModel),
                      controller: cardModel.textController,
                    ),
                  )
                  : DefaultTextStyle.merge(
                    style: cardLabelStyle.copyWith(fontSize: _varyFontSizes ? 5.0 + index : null),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: <Widget>[
                        Text(cardModel.textController.text, textAlign: _textAlign),
                      ],
                    ),
                  ),
        ),
      ),
    );

    String backgroundMessage;
    switch (_dismissDirection) {
      case DismissDirection.horizontal:
        backgroundMessage = 'Swipe in either direction';
      case DismissDirection.endToStart:
        backgroundMessage = 'Swipe left to dismiss';
      case DismissDirection.startToEnd:
        backgroundMessage = 'Swipe right to dismiss';
      case DismissDirection.vertical:
      case DismissDirection.up:
      case DismissDirection.down:
      case DismissDirection.none:
        backgroundMessage = 'Unsupported dismissDirection';
    }

    // This icon is wrong in RTL.
    Widget leftArrowIcon = const Icon(Icons.arrow_back, size: 36.0);
    if (_dismissDirection == DismissDirection.startToEnd) {
      leftArrowIcon = Opacity(opacity: 0.1, child: leftArrowIcon);
    }

    // This icon is wrong in RTL.
    Widget rightArrowIcon = const Icon(Icons.arrow_forward, size: 36.0);
    if (_dismissDirection == DismissDirection.endToStart) {
      rightArrowIcon = Opacity(opacity: 0.1, child: rightArrowIcon);
    }

    final ThemeData theme = Theme.of(context);
    final TextStyle? backgroundTextStyle = theme.primaryTextTheme.titleLarge;

    // The background Widget appears behind the Dismissible card when the card
    // moves to the left or right. The Positioned widget ensures that the
    // size of the background,card Stack will be based only on the card. The
    // Viewport ensures that when the card's resize animation occurs, the
    // background (text and icons) will just be clipped, not resized.
    final Widget background = Positioned.fill(
      child: Container(
        margin: const EdgeInsets.all(4.0),
        child: SingleChildScrollView(
          child: Container(
            height: cardModel.height,
            color: theme.primaryColor,
            child: Row(
              children: <Widget>[
                leftArrowIcon,
                Expanded(
                  child: Text(
                    backgroundMessage,
                    style: backgroundTextStyle,
                    textAlign: TextAlign.center,
                  ),
                ),
                rightArrowIcon,
              ],
            ),
          ),
        ),
      ),
    );

    return IconTheme(
      key: cardModel.key,
      data: const IconThemeData(color: Colors.white),
      child: Stack(children: <Widget>[background, card]),
    );
  }

  Shader _createShader(Rect bounds) {
    return const LinearGradient(
      begin: Alignment.topCenter,
      end: Alignment.bottomCenter,
      colors: <Color>[Color(0x00FFFFFF), Color(0xFFFFFFFF)],
      stops: <double>[0.1, 0.35],
    ).createShader(bounds);
  }

  @override
  Widget build(BuildContext context) {
    Widget cardCollection = ListView.builder(
      itemExtent: _fixedSizeCards ? kFixedCardHeight : null,
      itemCount: _cardModels.length,
      itemBuilder: _buildCard,
    );

    if (_sunshine) {
      cardCollection = Stack(
        children: <Widget>[
          Column(children: <Widget>[Image.network(_sunshineURL)]),
          ShaderMask(shaderCallback: _createShader, child: cardCollection),
        ],
      );
    }

    final Widget body = Container(
      padding: const EdgeInsets.symmetric(vertical: 12.0, horizontal: 8.0),
      color: _primaryColor.shade50,
      child: cardCollection,
    );

    return Theme(
      data: ThemeData(primarySwatch: _primaryColor),
      child: Scaffold(appBar: _buildAppBar(context), drawer: _buildDrawer(), body: body),
    );
  }
}

void main() {
  runApp(const MaterialApp(title: 'Cards', home: CardCollection()));
}
