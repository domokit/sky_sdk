// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

/// Flutter code sample for [SelectionArea].

void main() => runApp(const SelectionAreaColorTextRedExampleApp());

class SelectionAreaColorTextRedExampleApp extends StatelessWidget {
  const SelectionAreaColorTextRedExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        useMaterial3: true,
      ),
      home: const MyHomePage(title: 'Flutter Demo Home Page'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

typedef LocalSpanRange = ({int startOffset, int endOffset});

class _MyHomePageState extends State<MyHomePage> {
  final SelectionListenerNotifier _selectionNotifier = SelectionListenerNotifier();
  final ContextMenuController _menuController = ContextMenuController();
  final GlobalKey<SelectionAreaState> selectionAreaKey = GlobalKey<SelectionAreaState>();

  // The data of the top level TextSpans. Each TextSpan is mapped to a LocalSpanRange,
  // which is the range the textspan covers relative to the SelectionListener it is under.
  Map<LocalSpanRange, TextSpan> dataSourceMap = <LocalSpanRange, TextSpan>{};
  // The data of the bulleted list contained within a WidgetSpan. Each bullet is mapped
  // to a LocalSpanRange, being the range the bullet covers relative to the SelectionListener
  // it is under.
  Map<LocalSpanRange, TextSpan> bulletSourceMap = <LocalSpanRange, TextSpan>{};
  Map<int, Map<LocalSpanRange, TextSpan>> widgetSpanMaps = <int, Map<LocalSpanRange, TextSpan>>{};
  // The origin data used to restore the demo to its initial state.
  late final Map<LocalSpanRange, TextSpan> originSourceData;
  late final Map<LocalSpanRange, TextSpan> originBulletSourceData;

  void _initData() {
    const String bulletListTitle = 'This is some bulleted list:\n';
    final List<String> bullets = <String>[for (int i = 1; i <= 7; i += 1) '• Bullet $i'];
    final TextSpan bulletedList = TextSpan(
      text: bulletListTitle,
      children: <InlineSpan>[
        WidgetSpan(
          child: Column(
            children: <Widget>[
              for (final String bullet in bullets)
                Padding(padding: const EdgeInsets.only(left: 20.0), child: Text(bullet)),
            ],
          ),
        ),
      ],
    );

    int currentOffset = 0;
    // Map bulleted list span to a local range using its concrete length calculated
    // from the length of its title and each individual bullet.
    dataSourceMap[(
          startOffset: currentOffset,
          endOffset: bulletListTitle.length + bullets.join().length,
        )] =
        bulletedList;
    currentOffset += bulletListTitle.length;
    widgetSpanMaps[currentOffset] = bulletSourceMap;
    // Map individual bullets to a local range.
    for (final String bullet in bullets) {
      bulletSourceMap[(
        startOffset: currentOffset,
        endOffset: currentOffset + bullet.length,
      )] = TextSpan(text: bullet);
      currentOffset += bullet.length;
    }

    const TextSpan secondTextParagraph = TextSpan(
      text: 'This is some text in a text widget.',
      children: <InlineSpan>[TextSpan(text: ' This is some more text in the same text widget.')],
    );
    const TextSpan thirdTextParagraph = TextSpan(text: 'This is some text in another text widget.');
    // Map second and third paragraphs to local ranges.
    dataSourceMap[(
          startOffset: currentOffset,
          endOffset:
              currentOffset + secondTextParagraph.toPlainText(includeSemanticsLabels: false).length,
        )] =
        secondTextParagraph;
    currentOffset += secondTextParagraph.toPlainText(includeSemanticsLabels: false).length;
    dataSourceMap[(
          startOffset: currentOffset,
          endOffset:
              currentOffset + thirdTextParagraph.toPlainText(includeSemanticsLabels: false).length,
        )] =
        thirdTextParagraph;

    // Save the origin data so we can revert our changes.
    originSourceData = <LocalSpanRange, TextSpan>{};
    for (final MapEntry<LocalSpanRange, TextSpan> entry in dataSourceMap.entries) {
      originSourceData[entry.key] = entry.value;
    }
    originBulletSourceData = <LocalSpanRange, TextSpan>{};
    for (final MapEntry<LocalSpanRange, TextSpan> entry in bulletSourceMap.entries) {
      originBulletSourceData[entry.key] = entry.value;
    }
  }

  void _handleSelectableRegionStatusChanged(SelectableRegionSelectionStatus status) {
    if (_menuController.isShown) {
      ContextMenuController.removeAny();
    }
    if (_selectionNotifier.selection.status != SelectionStatus.uncollapsed ||
        status != SelectableRegionSelectionStatus.finalized) {
      return;
    }
    if (selectionAreaKey.currentState == null ||
        !selectionAreaKey.currentState!.mounted ||
        selectionAreaKey.currentState!.selectableRegion.contextMenuAnchors.secondaryAnchor ==
            null) {
      return;
    }
    final SelectedContentRange? selectedContentRange = _selectionNotifier.selection.range;
    if (selectedContentRange == null) {
      return;
    }
    _menuController.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return TapRegion(
          onTapOutside: (PointerDownEvent event) {
            if (_menuController.isShown) {
              ContextMenuController.removeAny();
            }
          },
          child: AdaptiveTextSelectionToolbar.buttonItems(
            buttonItems: <ContextMenuButtonItem>[
              ContextMenuButtonItem(
                onPressed: () {
                  ContextMenuController.removeAny();
                  _colorSelectionRed(
                    selectedContentRange,
                    dataMap: dataSourceMap,
                    coloringChildSpan: false,
                  );
                  selectionAreaKey.currentState!.selectableRegion.clearSelection();
                },
                label: 'Color Text Red',
              ),
            ],
            anchors: TextSelectionToolbarAnchors(
              primaryAnchor:
                  selectionAreaKey
                      .currentState!
                      .selectableRegion
                      .contextMenuAnchors
                      .secondaryAnchor!,
            ),
          ),
        );
      },
    );
  }

  void _colorSelectionRed(
    SelectedContentRange selectedContentRange, {
    required Map<LocalSpanRange, TextSpan> dataMap,
    required bool coloringChildSpan,
  }) {
    for (final MapEntry<LocalSpanRange, TextSpan> entry in dataMap.entries) {
      final LocalSpanRange entryLocalRange = entry.key;
      final int normalizedStartOffset = min(
        selectedContentRange.startOffset,
        selectedContentRange.endOffset,
      );
      final int normalizedEndOffset = max(
        selectedContentRange.startOffset,
        selectedContentRange.endOffset,
      );
      if (normalizedStartOffset > entryLocalRange.endOffset) {
        continue;
      }
      if (normalizedEndOffset < entryLocalRange.startOffset) {
        continue;
      }
      // The selection details is covering the current entry so let's color the range red.
      final TextSpan rawSpan = entry.value;
      // Determine local ranges relative to rawSpan.
      final int clampedLocalStart =
          normalizedStartOffset < entryLocalRange.startOffset
              ? entryLocalRange.startOffset
              : normalizedStartOffset;
      final int clampedLocalEnd =
          normalizedEndOffset > entryLocalRange.endOffset
              ? entryLocalRange.endOffset
              : normalizedEndOffset;
      final int startOffset = (clampedLocalStart - entryLocalRange.startOffset).abs();
      final int endOffset = startOffset + (clampedLocalEnd - clampedLocalStart).abs();
      final List<InlineSpan> beforeSelection = <InlineSpan>[];
      final List<InlineSpan> insideSelection = <InlineSpan>[];
      final List<InlineSpan> afterSelection = <InlineSpan>[];
      int count = 0;
      rawSpan.visitChildren((InlineSpan child) {
        if (child is TextSpan) {
          final String? rawText = child.text;
          if (rawText != null) {
            if (count < startOffset) {
              final int newStart = min(startOffset - count, rawText.length);
              final int globalNewStart = count + newStart;
              // Collect spans before selection.
              beforeSelection.add(
                TextSpan(style: child.style, text: rawText.substring(0, newStart)),
              );
              // Check if this span also contains the selection.
              if (globalNewStart == startOffset && newStart < rawText.length) {
                final int newStartAfterSelection = min(
                  newStart + (endOffset - startOffset),
                  rawText.length,
                );
                final int globalNewStartAfterSelection = count + newStartAfterSelection;
                insideSelection.add(
                  TextSpan(
                    style: const TextStyle(color: Colors.red).merge(entry.value.style),
                    text: rawText.substring(newStart, newStartAfterSelection),
                  ),
                );
                // Check if this span contains content after the selection.
                if (globalNewStartAfterSelection == endOffset &&
                    newStartAfterSelection < rawText.length) {
                  afterSelection.add(
                    TextSpan(style: child.style, text: rawText.substring(newStartAfterSelection)),
                  );
                }
              }
            } else if (count >= endOffset) {
              // Collect spans after selection.
              afterSelection.add(TextSpan(style: child.style, text: rawText));
            } else {
              // Collect spans inside selection.
              final int newStart = min(endOffset - count, rawText.length);
              final int globalNewStart = count + newStart;
              insideSelection.add(
                TextSpan(
                  style: const TextStyle(color: Colors.red),
                  text: rawText.substring(0, newStart),
                ),
              );
              // Check if this span contains content after the selection.
              if (globalNewStart == endOffset && newStart < rawText.length) {
                afterSelection.add(TextSpan(style: child.style, text: rawText.substring(newStart)));
              }
            }
            count += rawText.length;
          }
        } else if (child is WidgetSpan) {
          if (!widgetSpanMaps.containsKey(count)) {
            // We have arrived at a WidgetSpan but it is unaccounted for.
            return true;
          }
          final Map<LocalSpanRange, TextSpan> widgetSpanSourceMap = widgetSpanMaps[count]!;
          if (count < startOffset &&
              count +
                      (widgetSpanSourceMap.keys.last.endOffset -
                              widgetSpanSourceMap.keys.first.startOffset)
                          .abs() <
                  startOffset) {
            // When the count is less than the startOffset and we are at a widgetspan
            // it is still possible that the startOffset is somewhere within the widgetspan,
            // so we should try to color the selection red for the widgetspan.
            //
            // If the calculated widgetspan length would not extend the count past the
            // startOffset then add this widgetspan to the beforeSelection, and
            // continue walking the tree.
            beforeSelection.add(child);
            count +=
                (widgetSpanSourceMap.keys.last.endOffset -
                        widgetSpanSourceMap.keys.first.startOffset)
                    .abs();
            return true;
          } else if (count >= endOffset) {
            afterSelection.add(child);
            count +=
                (widgetSpanSourceMap.keys.last.endOffset -
                        widgetSpanSourceMap.keys.first.startOffset)
                    .abs();
            return true;
          }
          // Update widgetspan data.
          _colorSelectionRed(
            selectedContentRange,
            dataMap: widgetSpanSourceMap,
            coloringChildSpan: true,
          );
          // Re-create widgetspan.
          if (count == 28) {
            // The index where the bulleted list begins.
            insideSelection.add(
              WidgetSpan(
                child: Column(
                  children: <Widget>[
                    for (final MapEntry<LocalSpanRange, TextSpan> entry
                        in widgetSpanSourceMap.entries)
                      Padding(
                        padding: const EdgeInsets.only(left: 20.0),
                        child: Text.rich(widgetSpanSourceMap[entry.key]!),
                      ),
                  ],
                ),
              ),
            );
          }
          count +=
              (widgetSpanSourceMap.keys.last.endOffset - widgetSpanSourceMap.keys.first.startOffset)
                  .abs();
          return true;
        }
        return true;
      });
      dataMap[entry.key] = TextSpan(
        style: dataMap[entry.key]!.style,
        children: <InlineSpan>[...beforeSelection, ...insideSelection, ...afterSelection],
      );
    }
    // Avoid clearing the selection and setting the state
    // before we have colored all parts of the selection.
    if (!coloringChildSpan) {
      setState(() {});
    }
  }

  @override
  void initState() {
    super.initState();
    _initData();
  }

  @override
  void dispose() {
    _selectionNotifier.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: SelectionArea(
        key: selectionAreaKey,
        child: MySelectableTextColumn(
          selectionNotifier: _selectionNotifier,
          dataSourceMap: dataSourceMap,
          onChanged: _handleSelectableRegionStatusChanged,
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          setState(() {
            // Resets the state to the origin data.
            for (final MapEntry<LocalSpanRange, TextSpan> entry in originSourceData.entries) {
              dataSourceMap[entry.key] = entry.value;
            }
            for (final MapEntry<LocalSpanRange, TextSpan> entry in originBulletSourceData.entries) {
              bulletSourceMap[entry.key] = entry.value;
            }
          });
        },
        child: const Icon(Icons.undo),
      ),
    );
  }
}

class MySelectableTextColumn extends StatefulWidget {
  const MySelectableTextColumn({
    super.key,
    required this.selectionNotifier,
    required this.dataSourceMap,
    required this.onChanged,
  });

  final SelectionListenerNotifier selectionNotifier;
  final Map<LocalSpanRange, TextSpan> dataSourceMap;
  final ValueChanged<SelectableRegionSelectionStatus> onChanged;

  @override
  State<MySelectableTextColumn> createState() => _MySelectableTextColumnState();
}

class _MySelectableTextColumnState extends State<MySelectableTextColumn> {
  ValueListenable<SelectableRegionSelectionStatus>? _selectableRegionScope;

  void _handleOnSelectableRegionChanged() {
    if (_selectableRegionScope == null) {
      return;
    }
    widget.onChanged.call(_selectableRegionScope!.value);
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _selectableRegionScope?.removeListener(_handleOnSelectableRegionChanged);
    _selectableRegionScope = SelectableRegionSelectionStatusScope.maybeOf(context);
    _selectableRegionScope?.addListener(_handleOnSelectableRegionChanged);
  }

  @override
  void dispose() {
    _selectableRegionScope?.removeListener(_handleOnSelectableRegionChanged);
    _selectableRegionScope = null;
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SelectionListener(
      selectionNotifier: widget.selectionNotifier,
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            for (final MapEntry<LocalSpanRange, TextSpan> entry in widget.dataSourceMap.entries)
              Text.rich(entry.value),
          ],
        ),
      ),
    );
  }
}
