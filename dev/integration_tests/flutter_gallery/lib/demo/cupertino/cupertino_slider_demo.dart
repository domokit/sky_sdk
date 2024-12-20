// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

import '../../gallery/demo.dart';

class CupertinoSliderDemo extends StatefulWidget {
  const CupertinoSliderDemo({super.key});

  static const String routeName = '/cupertino/slider';

  @override
  State<CupertinoSliderDemo> createState() => _CupertinoSliderDemoState();
}

class _CupertinoSliderDemoState extends State<CupertinoSliderDemo> {
  double _value = 25.0;
  double _discreteValue = 20.0;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        middle: const Text('Sliders'),
        // We're specifying a back label here because the previous page is a
        // Material page. CupertinoPageRoutes could auto-populate these back
        // labels.
        previousPageTitle: 'Cupertino',
        trailing: CupertinoDemoDocumentationButton(CupertinoSliderDemo.routeName),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle,
        child: SafeArea(
          child: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: <Widget>[
                Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    CupertinoSlider(
                      value: _value,
                      max: 100.0,
                      onChanged: (double value) {
                        setState(() {
                          _value = value;
                        });
                      },
                    ),
                    Text('Cupertino Continuous: ${_value.toStringAsFixed(1)}'),
                  ],
                ),
                Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    CupertinoSlider(
                      value: _discreteValue,
                      max: 100.0,
                      divisions: 5,
                      onChanged: (double value) {
                        setState(() {
                          _discreteValue = value;
                        });
                      },
                    ),
                    Text('Cupertino Discrete: $_discreteValue'),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
