// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'stock_types.dart';

class StockSettings extends StatefulWidget {
  const StockSettings(this.configuration, this.updater, {super.key});

  final StockConfiguration configuration;
  final ValueChanged<StockConfiguration> updater;

  @override
  StockSettingsState createState() => StockSettingsState();
}

class StockSettingsState extends State<StockSettings> {
  void _handleOptimismChanged(bool? value) {
    value ??= false;
    sendUpdates(
      widget.configuration.copyWith(
        stockMode: value ? StockMode.optimistic : StockMode.pessimistic,
      ),
    );
  }

  void _handleBackupChanged(bool value) {
    sendUpdates(
      widget.configuration.copyWith(backupMode: value ? BackupMode.enabled : BackupMode.disabled),
    );
  }

  void _handleShowGridChanged(bool value) {
    sendUpdates(widget.configuration.copyWith(debugShowGrid: value));
  }

  void _handleShowSizesChanged(bool value) {
    sendUpdates(widget.configuration.copyWith(debugShowSizes: value));
  }

  void _handleShowBaselinesChanged(bool value) {
    sendUpdates(widget.configuration.copyWith(debugShowBaselines: value));
  }

  void _handleShowLayersChanged(bool value) {
    sendUpdates(widget.configuration.copyWith(debugShowLayers: value));
  }

  void _handleShowPointersChanged(bool value) {
    sendUpdates(widget.configuration.copyWith(debugShowPointers: value));
  }

  void _handleShowRainbowChanged(bool value) {
    sendUpdates(widget.configuration.copyWith(debugShowRainbow: value));
  }

  void _handleShowPerformanceOverlayChanged(bool value) {
    sendUpdates(widget.configuration.copyWith(showPerformanceOverlay: value));
  }

  void _handleShowSemanticsDebuggerChanged(bool value) {
    sendUpdates(widget.configuration.copyWith(showSemanticsDebugger: value));
  }

  void _confirmOptimismChange() {
    switch (widget.configuration.stockMode) {
      case StockMode.optimistic:
        _handleOptimismChanged(false);
      case StockMode.pessimistic:
        showDialog<bool>(
          context: context,
          builder: (BuildContext context) {
            return AlertDialog(
              title: const Text('Change mode?'),
              content: const Text(
                'Optimistic mode means everything is awesome. Are you sure you can handle that?',
              ),
              actions: <Widget>[
                TextButton(
                  child: const Text('NO THANKS'),
                  onPressed: () {
                    Navigator.pop(context, false);
                  },
                ),
                TextButton(
                  child: const Text('AGREE'),
                  onPressed: () {
                    Navigator.pop(context, true);
                  },
                ),
              ],
            );
          },
        ).then<void>(_handleOptimismChanged);
    }
  }

  void sendUpdates(StockConfiguration value) {
    widget.updater(value);
  }

  AppBar buildAppBar(BuildContext context) {
    return AppBar(title: const Text('Settings'));
  }

  Widget buildSettingsPane(BuildContext context) {
    final List<Widget> rows = <Widget>[
      ListTile(
        leading: const Icon(Icons.thumb_up),
        title: const Text('Everything is awesome'),
        onTap: _confirmOptimismChange,
        trailing: Checkbox(
          value: widget.configuration.stockMode == StockMode.optimistic,
          onChanged: (bool? value) => _confirmOptimismChange(),
        ),
      ),
      ListTile(
        leading: const Icon(Icons.backup),
        title: const Text('Back up stock list to the cloud'),
        onTap: () {
          _handleBackupChanged(!(widget.configuration.backupMode == BackupMode.enabled));
        },
        trailing: Switch(
          value: widget.configuration.backupMode == BackupMode.enabled,
          onChanged: _handleBackupChanged,
        ),
      ),
      ListTile(
        leading: const Icon(Icons.picture_in_picture),
        title: const Text('Show rendering performance overlay'),
        onTap: () {
          _handleShowPerformanceOverlayChanged(!widget.configuration.showPerformanceOverlay);
        },
        trailing: Switch(
          value: widget.configuration.showPerformanceOverlay,
          onChanged: _handleShowPerformanceOverlayChanged,
        ),
      ),
      ListTile(
        leading: const Icon(Icons.accessibility),
        title: const Text('Show semantics overlay'),
        onTap: () {
          _handleShowSemanticsDebuggerChanged(!widget.configuration.showSemanticsDebugger);
        },
        trailing: Switch(
          value: widget.configuration.showSemanticsDebugger,
          onChanged: _handleShowSemanticsDebuggerChanged,
        ),
      ),
    ];
    assert(() {
      // material grid and size construction lines are only available in debug mode
      rows.addAll(<Widget>[
        ListTile(
          leading: const Icon(Icons.border_clear),
          title: const Text('Show material grid (for debugging)'),
          onTap: () {
            _handleShowGridChanged(!widget.configuration.debugShowGrid);
          },
          trailing: Switch(
            value: widget.configuration.debugShowGrid,
            onChanged: _handleShowGridChanged,
          ),
        ),
        ListTile(
          leading: const Icon(Icons.border_all),
          title: const Text('Show construction lines (for debugging)'),
          onTap: () {
            _handleShowSizesChanged(!widget.configuration.debugShowSizes);
          },
          trailing: Switch(
            value: widget.configuration.debugShowSizes,
            onChanged: _handleShowSizesChanged,
          ),
        ),
        ListTile(
          leading: const Icon(Icons.format_color_text),
          title: const Text('Show baselines (for debugging)'),
          onTap: () {
            _handleShowBaselinesChanged(!widget.configuration.debugShowBaselines);
          },
          trailing: Switch(
            value: widget.configuration.debugShowBaselines,
            onChanged: _handleShowBaselinesChanged,
          ),
        ),
        ListTile(
          leading: const Icon(Icons.filter_none),
          title: const Text('Show layer boundaries (for debugging)'),
          onTap: () {
            _handleShowLayersChanged(!widget.configuration.debugShowLayers);
          },
          trailing: Switch(
            value: widget.configuration.debugShowLayers,
            onChanged: _handleShowLayersChanged,
          ),
        ),
        ListTile(
          leading: const Icon(Icons.mouse),
          title: const Text('Show pointer hit-testing (for debugging)'),
          onTap: () {
            _handleShowPointersChanged(!widget.configuration.debugShowPointers);
          },
          trailing: Switch(
            value: widget.configuration.debugShowPointers,
            onChanged: _handleShowPointersChanged,
          ),
        ),
        ListTile(
          leading: const Icon(Icons.gradient),
          title: const Text('Show repaint rainbow (for debugging)'),
          onTap: () {
            _handleShowRainbowChanged(!widget.configuration.debugShowRainbow);
          },
          trailing: Switch(
            value: widget.configuration.debugShowRainbow,
            onChanged: _handleShowRainbowChanged,
          ),
        ),
      ]);
      return true;
    }());
    return ListView(padding: const EdgeInsets.symmetric(vertical: 20.0), children: rows);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(appBar: buildAppBar(context), body: buildSettingsPane(context));
  }
}
