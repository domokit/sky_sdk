// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

import '../../gallery/demo.dart';

const Color _kKeyUmbraOpacity = Color(0x33000000); // alpha = 0.2
const Color _kKeyPenumbraOpacity = Color(0x24000000); // alpha = 0.14
const Color _kAmbientShadowOpacity = Color(0x1F000000); // alpha = 0.12

class CupertinoSegmentedControlDemo extends StatefulWidget {
  const CupertinoSegmentedControlDemo({super.key});

  static const String routeName = 'cupertino/segmented_control';

  @override
  State<CupertinoSegmentedControlDemo> createState() => _CupertinoSegmentedControlDemoState();
}

class _CupertinoSegmentedControlDemoState extends State<CupertinoSegmentedControlDemo> {
  final Map<int, Widget> children = const <int, Widget>{
    0: Text('Small'),
    1: Text('Medium'),
    2: Text('Large'),
  };

  final Map<int, Widget> icons = const <int, Widget>{
    0: Center(child: FlutterLogo(size: 100.0)),
    1: Center(child: FlutterLogo(size: 200.0)),
    2: Center(child: FlutterLogo(size: 300.0)),
  };

  int? currentSegment = 0;

  void onValueChanged(int? newValue) {
    setState(() {
      currentSegment = newValue;
    });
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        middle: const Text('Segmented Control'),
        // We're specifying a back label here because the previous page is a
        // Material page. CupertinoPageRoutes could auto-populate these back
        // labels.
        previousPageTitle: 'Cupertino',
        trailing: CupertinoDemoDocumentationButton(CupertinoSegmentedControlDemo.routeName),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle.copyWith(fontSize: 13),
        child: SafeArea(
          child: Column(
            children: <Widget>[
              const Padding(padding: EdgeInsets.all(16.0)),
              SizedBox(
                width: 500.0,
                child: CupertinoSegmentedControl<int>(
                  children: children,
                  onValueChanged: onValueChanged,
                  groupValue: currentSegment,
                ),
              ),
              SizedBox(
                width: 500,
                child: Padding(
                  padding: const EdgeInsets.all(16.0),
                  child: CupertinoSlidingSegmentedControl<int>(
                    children: children,
                    onValueChanged: onValueChanged,
                    groupValue: currentSegment,
                  ),
                ),
              ),
              Expanded(
                child: Padding(
                  padding: const EdgeInsets.symmetric(vertical: 32.0, horizontal: 16.0),
                  child: CupertinoUserInterfaceLevel(
                    data: CupertinoUserInterfaceLevelData.elevated,
                    child: Builder(
                      builder: (BuildContext context) {
                        return Container(
                          padding: const EdgeInsets.symmetric(vertical: 64.0, horizontal: 16.0),
                          decoration: BoxDecoration(
                            color: CupertinoTheme.of(context).scaffoldBackgroundColor,
                            borderRadius: BorderRadius.circular(3.0),
                            boxShadow: const <BoxShadow>[
                              BoxShadow(
                                offset: Offset(0.0, 3.0),
                                blurRadius: 5.0,
                                spreadRadius: -1.0,
                                color: _kKeyUmbraOpacity,
                              ),
                              BoxShadow(
                                offset: Offset(0.0, 6.0),
                                blurRadius: 10.0,
                                color: _kKeyPenumbraOpacity,
                              ),
                              BoxShadow(
                                offset: Offset(0.0, 1.0),
                                blurRadius: 18.0,
                                color: _kAmbientShadowOpacity,
                              ),
                            ],
                          ),
                          child: icons[currentSegment!],
                        );
                      },
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
