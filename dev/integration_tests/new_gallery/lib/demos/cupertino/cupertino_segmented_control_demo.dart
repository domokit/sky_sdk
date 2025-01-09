// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

import '../../gallery_localizations.dart';

// BEGIN cupertinoSegmentedControlDemo

class CupertinoSegmentedControlDemo extends StatefulWidget {
  const CupertinoSegmentedControlDemo({super.key});

  @override
  State<CupertinoSegmentedControlDemo> createState() => _CupertinoSegmentedControlDemoState();
}

class _CupertinoSegmentedControlDemoState extends State<CupertinoSegmentedControlDemo>
    with RestorationMixin {
  RestorableInt currentSegment = RestorableInt(0);

  @override
  String get restorationId => 'cupertino_segmented_control';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(currentSegment, 'current_segment');
  }

  void onValueChanged(int? newValue) {
    setState(() {
      currentSegment.value = newValue!;
    });
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    const double segmentedControlMaxWidth = 500.0;
    final Map<int, Widget> children = <int, Widget>{
      0: Text(localizations.colorsIndigo),
      1: Text(localizations.colorsTeal),
      2: Text(localizations.colorsCyan),
    };

    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(localizations.demoCupertinoSegmentedControlTitle),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle.copyWith(fontSize: 13),
        child: SafeArea(
          child: ListView(
            children: <Widget>[
              const SizedBox(height: 16),
              SizedBox(
                width: segmentedControlMaxWidth,
                child: CupertinoSegmentedControl<int>(
                  children: children,
                  onValueChanged: onValueChanged,
                  groupValue: currentSegment.value,
                ),
              ),
              SizedBox(
                width: segmentedControlMaxWidth,
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: CupertinoSlidingSegmentedControl<int>(
                    children: children,
                    onValueChanged: onValueChanged,
                    groupValue: currentSegment.value,
                  ),
                ),
              ),
              Container(
                padding: const EdgeInsets.all(16),
                height: 300,
                alignment: Alignment.center,
                child: children[currentSegment.value],
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// END
