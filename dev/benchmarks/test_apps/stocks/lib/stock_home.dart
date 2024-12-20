// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart'
    show debugDumpLayerTree, debugDumpRenderTree, debugDumpSemanticsTree;
import 'package:flutter/scheduler.dart' show timeDilation;

import 'i18n/stock_strings.dart';
import 'stock_data.dart';
import 'stock_list.dart';
import 'stock_symbol_viewer.dart';
import 'stock_types.dart';

typedef ModeUpdater = void Function(StockMode mode);

enum _StockMenuItem { autorefresh, refresh, speedUp, speedDown }

enum StockHomeTab { market, portfolio }

class _NotImplementedDialog extends StatelessWidget {
  const _NotImplementedDialog();

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('Not Implemented'),
      content: const Text('This feature has not yet been implemented.'),
      actions: <Widget>[
        TextButton(
          onPressed: debugDumpApp,
          child: Row(
            children: <Widget>[
              const Icon(Icons.dvr, size: 18.0),
              Container(width: 8.0),
              const Text('DUMP APP TO CONSOLE'),
            ],
          ),
        ),
        TextButton(
          onPressed: () {
            Navigator.pop(context, false);
          },
          child: const Text('OH WELL'),
        ),
      ],
    );
  }
}

class StockHome extends StatefulWidget {
  const StockHome(this.stocks, this.configuration, this.updater, {super.key});

  final StockData stocks;
  final StockConfiguration configuration;
  final ValueChanged<StockConfiguration> updater;

  @override
  StockHomeState createState() => StockHomeState();
}

class StockHomeState extends State<StockHome> {
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();
  final TextEditingController _searchQuery = TextEditingController();
  bool _isSearching = false;
  bool _autorefresh = false;

  void _handleSearchBegin() {
    ModalRoute.of(context)!.addLocalHistoryEntry(
      LocalHistoryEntry(
        onRemove: () {
          setState(() {
            _isSearching = false;
            _searchQuery.clear();
          });
        },
      ),
    );
    setState(() {
      _isSearching = true;
    });
  }

  void _handleStockModeChange(StockMode? value) {
    widget.updater(widget.configuration.copyWith(stockMode: value));
  }

  void _handleStockMenu(BuildContext context, _StockMenuItem value) {
    switch (value) {
      case _StockMenuItem.autorefresh:
        setState(() {
          _autorefresh = !_autorefresh;
        });
      case _StockMenuItem.refresh:
        showDialog<void>(
          context: context,
          builder: (BuildContext context) => const _NotImplementedDialog(),
        );
      case _StockMenuItem.speedUp:
        timeDilation /= 5.0;
      case _StockMenuItem.speedDown:
        timeDilation *= 5.0;
    }
  }

  Widget _buildDrawer(BuildContext context) {
    return Drawer(
      child: ListView(
        dragStartBehavior: DragStartBehavior.down,
        children: <Widget>[
          const DrawerHeader(child: Center(child: Text('Stocks'))),
          const ListTile(
            leading: Icon(Icons.assessment),
            title: Text('Stock List'),
            selected: true,
          ),
          const ListTile(
            leading: Icon(Icons.account_balance),
            title: Text('Account Balance'),
            enabled: false,
          ),
          ListTile(
            leading: const Icon(Icons.dvr),
            title: const Text('Dump App to Console'),
            onTap: () {
              try {
                debugDumpApp();
                debugDumpRenderTree();
                debugDumpLayerTree();
                debugDumpSemanticsTree();
              } catch (e, stack) {
                debugPrint('Exception while dumping app:\n$e\n$stack');
              }
            },
          ),
          const Divider(),
          ListTile(
            leading: const Icon(Icons.thumb_up),
            title: const Text('Optimistic'),
            trailing: Radio<StockMode>(
              value: StockMode.optimistic,
              groupValue: widget.configuration.stockMode,
              onChanged: _handleStockModeChange,
            ),
            onTap: () {
              _handleStockModeChange(StockMode.optimistic);
            },
          ),
          ListTile(
            leading: const Icon(Icons.thumb_down),
            title: const Text('Pessimistic'),
            trailing: Radio<StockMode>(
              value: StockMode.pessimistic,
              groupValue: widget.configuration.stockMode,
              onChanged: _handleStockModeChange,
            ),
            onTap: () {
              _handleStockModeChange(StockMode.pessimistic);
            },
          ),
          const Divider(),
          ListTile(
            leading: const Icon(Icons.settings),
            title: const Text('Settings'),
            onTap: _handleShowSettings,
          ),
          ListTile(
            leading: const Icon(Icons.help),
            title: const Text('About'),
            onTap: _handleShowAbout,
          ),
        ],
      ),
    );
  }

  void _handleShowSettings() {
    Navigator.popAndPushNamed(context, '/settings');
  }

  void _handleShowAbout() {
    showAboutDialog(context: context);
  }

  AppBar buildAppBar() {
    return AppBar(
      elevation: 0.0,
      title: Text(StockStrings.of(context).title),
      actions: <Widget>[
        IconButton(
          icon: const Icon(Icons.search),
          onPressed: _handleSearchBegin,
          tooltip: 'Search',
        ),
        PopupMenuButton<_StockMenuItem>(
          onSelected: (_StockMenuItem value) {
            _handleStockMenu(context, value);
          },
          itemBuilder:
              (BuildContext context) => <PopupMenuItem<_StockMenuItem>>[
                CheckedPopupMenuItem<_StockMenuItem>(
                  value: _StockMenuItem.autorefresh,
                  checked: _autorefresh,
                  child: const Text('Autorefresh'),
                ),
                const PopupMenuItem<_StockMenuItem>(
                  value: _StockMenuItem.refresh,
                  child: Text('Refresh'),
                ),
                const PopupMenuItem<_StockMenuItem>(
                  value: _StockMenuItem.speedUp,
                  child: Text('Increase animation speed'),
                ),
                const PopupMenuItem<_StockMenuItem>(
                  value: _StockMenuItem.speedDown,
                  child: Text('Decrease animation speed'),
                ),
              ],
        ),
      ],
      bottom: TabBar(
        tabs: <Widget>[
          Tab(text: StockStrings.of(context).market),
          Tab(text: StockStrings.of(context).portfolio),
        ],
      ),
    );
  }

  static Iterable<Stock> _getStockList(StockData stocks, Iterable<String> symbols) {
    return symbols
        .map<Stock?>((String symbol) => stocks[symbol])
        .where((Stock? stock) => stock != null)
        .cast<Stock>();
  }

  Iterable<Stock> _filterBySearchQuery(Iterable<Stock> stocks) {
    if (_searchQuery.text.isEmpty) {
      return stocks;
    }
    final RegExp regexp = RegExp(_searchQuery.text, caseSensitive: false);
    return stocks.where((Stock stock) => stock.symbol.contains(regexp));
  }

  void _buyStock(Stock stock) {
    setState(() {
      stock.percentChange = 100.0 * (1.0 / stock.lastSale);
      stock.lastSale += 1.0;
    });
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Purchased ${stock.symbol} for ${stock.lastSale}'),
        action: SnackBarAction(
          label: 'BUY MORE',
          onPressed: () {
            _buyStock(stock);
          },
        ),
      ),
    );
  }

  Widget _buildStockList(BuildContext context, Iterable<Stock> stocks, StockHomeTab tab) {
    return StockList(
      stocks: stocks.toList(),
      onAction: _buyStock,
      onOpen: (Stock stock) {
        Navigator.pushNamed(context, '/stock', arguments: stock.symbol);
      },
      onShow: (Stock stock) {
        _scaffoldKey.currentState!.showBottomSheet(
          (BuildContext context) => StockSymbolBottomSheet(stock: stock),
        );
      },
    );
  }

  Widget _buildStockTab(BuildContext context, StockHomeTab tab, List<String> stockSymbols) {
    return AnimatedBuilder(
      key: ValueKey<StockHomeTab>(tab),
      animation: Listenable.merge(<Listenable>[_searchQuery, widget.stocks]),
      builder: (BuildContext context, Widget? child) {
        return _buildStockList(
          context,
          _filterBySearchQuery(_getStockList(widget.stocks, stockSymbols)).toList(),
          tab,
        );
      },
    );
  }

  static const List<String> portfolioSymbols = <String>[
    'AAPL',
    'FIZZ',
    'FIVE',
    'FLAT',
    'ZINC',
    'ZNGA',
  ];

  AppBar buildSearchBar() {
    return AppBar(
      leading: BackButton(color: Theme.of(context).colorScheme.secondary),
      title: TextField(
        controller: _searchQuery,
        autofocus: true,
        decoration: const InputDecoration(hintText: 'Search stocks'),
      ),
      backgroundColor: Theme.of(context).canvasColor,
    );
  }

  void _handleCreateCompany() {
    showModalBottomSheet<void>(
      context: context,
      builder: (BuildContext context) => const _CreateCompanySheet(),
    );
  }

  Widget buildFloatingActionButton() {
    return FloatingActionButton(
      tooltip: 'Create company',
      backgroundColor: Theme.of(context).colorScheme.secondary,
      onPressed: _handleCreateCompany,
      child: const Icon(Icons.add),
    );
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: 2,
      child: Scaffold(
        drawerDragStartBehavior: DragStartBehavior.down,
        key: _scaffoldKey,
        appBar: _isSearching ? buildSearchBar() : buildAppBar(),
        floatingActionButton: buildFloatingActionButton(),
        drawer: _buildDrawer(context),
        body: TabBarView(
          dragStartBehavior: DragStartBehavior.down,
          children: <Widget>[
            _buildStockTab(context, StockHomeTab.market, widget.stocks.allSymbols),
            _buildStockTab(context, StockHomeTab.portfolio, portfolioSymbols),
          ],
        ),
      ),
    );
  }
}

class _CreateCompanySheet extends StatelessWidget {
  const _CreateCompanySheet();

  @override
  Widget build(BuildContext context) {
    return const Column(
      children: <Widget>[
        TextField(autofocus: true, decoration: InputDecoration(hintText: 'Company Name')),
        Text('(This demo is not yet complete.)'),
        // For example, we could add a button that actually updates the list
        // and then contacts the server, etc.
      ],
    );
  }
}
