// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SearchAnchor.bar].

void main() => runApp(const SearchBarApp());

class SearchBarApp extends StatefulWidget {
  const SearchBarApp({super.key});

  @override
  State<SearchBarApp> createState() => _SearchBarAppState();
}

class _SearchBarAppState extends State<SearchBarApp> {
  Color? selectedColorSeed;
  List<ColorLabel> searchHistory = <ColorLabel>[];

  Iterable<Widget> getHistoryList(SearchController controller) {
    return searchHistory.map(
      (ColorLabel color) => ListTile(
        leading: const Icon(Icons.history),
        title: Text(color.label),
        trailing: IconButton(
          icon: const Icon(Icons.call_missed),
          onPressed: () {
            controller.text = color.label;
            controller.selection = TextSelection.collapsed(offset: controller.text.length);
          },
        ),
      ),
    );
  }

  Iterable<Widget> getSuggestions(SearchController controller) {
    final String input = controller.value.text;
    return ColorLabel.values
        .where((ColorLabel color) => color.label.contains(input))
        .map(
          (ColorLabel filteredColor) => ListTile(
            leading: CircleAvatar(backgroundColor: filteredColor.color),
            title: Text(filteredColor.label),
            trailing: IconButton(
              icon: const Icon(Icons.call_missed),
              onPressed: () {
                controller.text = filteredColor.label;
                controller.selection = TextSelection.collapsed(offset: controller.text.length);
              },
            ),
            onTap: () {
              controller.closeView(filteredColor.label);
              handleSelection(filteredColor);
            },
          ),
        );
  }

  void handleSelection(ColorLabel selectedColor) {
    setState(() {
      selectedColorSeed = selectedColor.color;
      if (searchHistory.length >= 5) {
        searchHistory.removeLast();
      }
      searchHistory.insert(0, selectedColor);
    });
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData themeData = ThemeData(colorSchemeSeed: selectedColorSeed);
    final ColorScheme colors = themeData.colorScheme;

    return MaterialApp(
      theme: themeData,
      home: Scaffold(
        appBar: AppBar(title: const Text('Search Bar Sample')),
        body: Align(
          alignment: Alignment.topCenter,
          child: Column(
            children: <Widget>[
              SearchAnchor.bar(
                barHintText: 'Search colors',
                suggestionsBuilder: (BuildContext context, SearchController controller) {
                  if (controller.text.isEmpty) {
                    if (searchHistory.isNotEmpty) {
                      return getHistoryList(controller);
                    }
                    return <Widget>[
                      Center(
                        child: Text('No search history.', style: TextStyle(color: colors.outline)),
                      ),
                    ];
                  }
                  return getSuggestions(controller);
                },
              ),
              cardSize,
              Card(color: colors.primary, child: cardSize),
              Card(color: colors.onPrimary, child: cardSize),
              Card(color: colors.primaryContainer, child: cardSize),
              Card(color: colors.onPrimaryContainer, child: cardSize),
              Card(color: colors.secondary, child: cardSize),
              Card(color: colors.onSecondary, child: cardSize),
            ],
          ),
        ),
      ),
    );
  }
}

SizedBox cardSize = const SizedBox(width: 80, height: 30);

enum ColorLabel {
  red('red', Colors.red),
  orange('orange', Colors.orange),
  yellow('yellow', Colors.yellow),
  green('green', Colors.green),
  blue('blue', Colors.blue),
  indigo('indigo', Colors.indigo),
  violet('violet', Color(0xFF8F00FF)),
  purple('purple', Colors.purple),
  pink('pink', Colors.pink),
  silver('silver', Color(0xFF808080)),
  gold('gold', Color(0xFFFFD700)),
  beige('beige', Color(0xFFF5F5DC)),
  brown('brown', Colors.brown),
  grey('grey', Colors.grey),
  black('black', Colors.black),
  white('white', Colors.white);

  const ColorLabel(this.label, this.color);
  final String label;
  final Color color;
}
