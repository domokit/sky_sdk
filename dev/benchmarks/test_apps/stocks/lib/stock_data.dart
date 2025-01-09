// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Snapshot from http://www.nasdaq.com/screening/company-list.aspx
// Fetched 2/23/2014.
// "Symbol","Name","LastSale","MarketCap","IPOyear","Sector","industry","Summary Quote",
// Data in stock_data.json

import 'dart:convert';
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;

final math.Random _rng = math.Random();

class Stock {
  Stock(this.symbol, this.name, this.lastSale, this.marketCap, this.percentChange);

  Stock.fromFields(List<String> fields) {
    // TODO(jackson): This class should only have static data, not lastSale, etc.
    // "Symbol","Name","LastSale","MarketCap","IPOyear","Sector","industry","Summary Quote",
    lastSale = 0.0;
    try {
      lastSale = double.parse(fields[2]);
    } catch (_) {}
    symbol = fields[0];
    name = fields[1];
    marketCap = fields[4];
    percentChange = (_rng.nextDouble() * 20) - 10;
  }

  late String symbol;
  late String name;
  late double lastSale;
  late String marketCap;
  late double percentChange;
}

class StockData extends ChangeNotifier {
  StockData() {
    if (actuallyFetchData) {
      _httpClient = http.Client();
      _fetchNextChunk();
    }
  }

  final List<String> _symbols = <String>[];
  final Map<String, Stock> _stocks = <String, Stock>{};

  List<String> get allSymbols => _symbols;

  Stock? operator [](String symbol) => _stocks[symbol];

  bool get loading => _httpClient != null;

  void add(List<dynamic> data) {
    for (final List<dynamic> fields in data.cast<List<dynamic>>()) {
      final Stock stock = Stock.fromFields(fields.cast<String>());
      _symbols.add(stock.symbol);
      _stocks[stock.symbol] = stock;
    }
    _symbols.sort();
    notifyListeners();
  }

  static const int _chunkCount = 30;
  int _nextChunk = 0;

  Uri _urlToFetch(int chunk) =>
      Uri.https('domokit.github.io', 'examples/stocks/data/stock_data_$chunk.json');

  http.Client? _httpClient;

  static bool actuallyFetchData = true;

  void _fetchNextChunk() {
    _httpClient!.get(_urlToFetch(_nextChunk++)).then<void>((http.Response response) {
      final String json = response.body;
      const JsonDecoder decoder = JsonDecoder();
      add(decoder.convert(json) as List<dynamic>);
      if (_nextChunk < _chunkCount) {
        _fetchNextChunk();
      } else {
        _end();
      }
    });
  }

  void _end() {
    _httpClient!.close();
    _httpClient = null;
  }
}
