// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

enum StockMode { optimistic, pessimistic }

enum BackupMode { enabled, disabled }

class StockConfiguration {
  StockConfiguration({
    required this.stockMode,
    required this.backupMode,
    required this.debugShowGrid,
    required this.debugShowSizes,
    required this.debugShowBaselines,
    required this.debugShowLayers,
    required this.debugShowPointers,
    required this.debugShowRainbow,
    required this.showPerformanceOverlay,
    required this.showSemanticsDebugger,
  });

  final StockMode stockMode;
  final BackupMode backupMode;
  final bool debugShowGrid;
  final bool debugShowSizes;
  final bool debugShowBaselines;
  final bool debugShowLayers;
  final bool debugShowPointers;
  final bool debugShowRainbow;
  final bool showPerformanceOverlay;
  final bool showSemanticsDebugger;

  StockConfiguration copyWith({
    StockMode? stockMode,
    BackupMode? backupMode,
    bool? debugShowGrid,
    bool? debugShowSizes,
    bool? debugShowBaselines,
    bool? debugShowLayers,
    bool? debugShowPointers,
    bool? debugShowRainbow,
    bool? showPerformanceOverlay,
    bool? showSemanticsDebugger,
  }) {
    return StockConfiguration(
      stockMode: stockMode ?? this.stockMode,
      backupMode: backupMode ?? this.backupMode,
      debugShowGrid: debugShowGrid ?? this.debugShowGrid,
      debugShowSizes: debugShowSizes ?? this.debugShowSizes,
      debugShowBaselines: debugShowBaselines ?? this.debugShowBaselines,
      debugShowLayers: debugShowLayers ?? this.debugShowLayers,
      debugShowPointers: debugShowPointers ?? this.debugShowPointers,
      debugShowRainbow: debugShowRainbow ?? this.debugShowRainbow,
      showPerformanceOverlay: showPerformanceOverlay ?? this.showPerformanceOverlay,
      showSemanticsDebugger: showSemanticsDebugger ?? this.showSemanticsDebugger,
    );
  }
}
