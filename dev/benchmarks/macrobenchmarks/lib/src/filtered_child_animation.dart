// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';

enum FilterType { opacity, rotateTransform, rotateFilter }

class FilteredChildAnimationPage extends StatefulWidget {
  const FilteredChildAnimationPage(
    this.initialFilterType, {
    super.key,
    this.initialComplexChild = true,
    this.initialUseRepaintBoundary = true,
  });

  final FilterType initialFilterType;
  final bool initialComplexChild;
  final bool initialUseRepaintBoundary;

  @override
  State<FilteredChildAnimationPage> createState() => _FilteredChildAnimationPageState();
}

class _FilteredChildAnimationPageState extends State<FilteredChildAnimationPage>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  final GlobalKey _childKey = GlobalKey(debugLabel: 'child to animate');
  Offset _childCenter = Offset.zero;

  FilterType? _filterType;
  late bool _complexChild;
  late bool _useRepaintBoundary;

  @override
  void initState() {
    super.initState();
    _filterType = widget.initialFilterType;
    _complexChild = widget.initialComplexChild;
    _useRepaintBoundary = widget.initialUseRepaintBoundary;
    WidgetsBinding.instance.addPostFrameCallback((_) {
      final RenderBox childBox = _childKey.currentContext!.findRenderObject()! as RenderBox;
      _childCenter = childBox.paintBounds.center;
    });
    _controller = AnimationController(vsync: this, duration: const Duration(seconds: 2));
    _controller.repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  void _setFilterType(FilterType type, bool selected) {
    setState(() => _filterType = selected ? type : null);
  }

  String get _title => switch (_filterType) {
    FilterType.opacity => 'Fading Child Animation',
    FilterType.rotateTransform => 'Transformed Child Animation',
    FilterType.rotateFilter => 'Matrix Filtered Child Animation',
    null => 'Static Child',
  };

  static Widget _makeChild(int rows, int cols, double fontSize, bool complex) {
    final BoxDecoration decoration = BoxDecoration(
      color: Colors.green,
      boxShadow: complex ? <BoxShadow>[const BoxShadow(blurRadius: 10.0)] : null,
      borderRadius: BorderRadius.circular(10.0),
    );
    return Stack(
      alignment: Alignment.center,
      children: <Widget>[
        Column(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: List<Widget>.generate(
            rows,
            (int r) => Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: List<Widget>.generate(
                cols,
                (int c) => Container(
                  decoration: decoration,
                  child: Text('text', style: TextStyle(fontSize: fontSize)),
                ),
              ),
            ),
          ),
        ),
        const Text('child', style: TextStyle(color: Colors.blue, fontSize: 36)),
      ],
    );
  }

  Widget _animate({required Widget child, required bool protectChild}) {
    if (_filterType == null) {
      _controller.reset();
      return child;
    }
    final FilterType filterType = _filterType!;
    _controller.repeat();
    Widget Function(BuildContext, Widget?) builder;
    switch (filterType) {
      case FilterType.opacity:
        builder =
            (BuildContext context, Widget? child) =>
                Opacity(opacity: (_controller.value * 2.0 - 1.0).abs(), child: child);
      case FilterType.rotateTransform:
        builder =
            (BuildContext context, Widget? child) => Transform(
              transform: Matrix4.rotationZ(_controller.value * 2.0 * pi),
              alignment: Alignment.center,
              filterQuality: FilterQuality.low,
              child: child,
            );
      case FilterType.rotateFilter:
        builder =
            (BuildContext context, Widget? child) => ImageFiltered(
              imageFilter: ImageFilter.matrix(
                (Matrix4.identity()
                      ..translate(_childCenter.dx, _childCenter.dy)
                      ..rotateZ(_controller.value * 2.0 * pi)
                      ..translate(-_childCenter.dx, -_childCenter.dy))
                    .storage,
              ),
              child: child,
            );
    }
    return RepaintBoundary(
      child: AnimatedBuilder(
        animation: _controller,
        builder: builder,
        child: protectChild ? RepaintBoundary(child: child) : child,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(_title)),
      body: Center(
        child: _animate(
          child: Container(
            key: _childKey,
            color: Colors.yellow,
            width: 300,
            height: 300,
            child: Center(child: _makeChild(4, 3, 24.0, _complexChild)),
          ),
          protectChild: _useRepaintBoundary,
        ),
      ),
      bottomNavigationBar: BottomAppBar(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                const Text('Opacity:'),
                Checkbox(
                  value: _filterType == FilterType.opacity,
                  onChanged: (bool? b) => _setFilterType(FilterType.opacity, b ?? false),
                ),
                const Text('Tx Rotate:'),
                Checkbox(
                  value: _filterType == FilterType.rotateTransform,
                  onChanged: (bool? b) => _setFilterType(FilterType.rotateTransform, b ?? false),
                ),
                const Text('IF Rotate:'),
                Checkbox(
                  value: _filterType == FilterType.rotateFilter,
                  onChanged: (bool? b) => _setFilterType(FilterType.rotateFilter, b ?? false),
                ),
              ],
            ),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                const Text('Complex child:'),
                Checkbox(
                  value: _complexChild,
                  onChanged: (bool? b) => setState(() => _complexChild = b ?? false),
                ),
                const Text('RPB on child:'),
                Checkbox(
                  value: _useRepaintBoundary,
                  onChanged: (bool? b) => setState(() => _useRepaintBoundary = b ?? false),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
