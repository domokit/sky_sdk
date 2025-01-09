// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import '../../gallery_localizations.dart';

// BEGIN cupertinoSliderDemo

class CupertinoSliderDemo extends StatefulWidget {
  const CupertinoSliderDemo({super.key});

  @override
  State<CupertinoSliderDemo> createState() => _CupertinoSliderDemoState();
}

class _CupertinoSliderDemoState extends State<CupertinoSliderDemo> with RestorationMixin {
  final RestorableDouble _value = RestorableDouble(25.0);
  final RestorableDouble _discreteValue = RestorableDouble(20.0);

  @override
  String get restorationId => 'cupertino_slider_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_value, 'value');
    registerForRestoration(_discreteValue, 'discrete_value');
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(localizations.demoCupertinoSliderTitle),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle,
        child: Center(
          child: Wrap(
            children: <Widget>[
              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  const SizedBox(height: 32),
                  CupertinoSlider(
                    value: _value.value,
                    max: 100.0,
                    onChanged: (double value) {
                      setState(() {
                        _value.value = value;
                      });
                    },
                  ),
                  CupertinoSlider(value: _value.value, max: 100.0, onChanged: null),
                  MergeSemantics(
                    child: Text(
                      localizations.demoCupertinoSliderContinuous(_value.value.toStringAsFixed(1)),
                    ),
                  ),
                ],
              ),
              // Disabled sliders
              // TODO(guidezpl): See https://github.com/flutter/flutter/issues/106691
              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  const SizedBox(height: 32),
                  CupertinoSlider(
                    value: _discreteValue.value,
                    max: 100.0,
                    divisions: 5,
                    onChanged: (double value) {
                      setState(() {
                        _discreteValue.value = value;
                      });
                    },
                  ),
                  CupertinoSlider(
                    value: _discreteValue.value,
                    max: 100.0,
                    divisions: 5,
                    onChanged: null,
                  ),
                  MergeSemantics(
                    child: Text(
                      localizations.demoCupertinoSliderDiscrete(
                        _discreteValue.value.toStringAsFixed(1),
                      ),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// END
