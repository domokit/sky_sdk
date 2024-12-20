// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_gallery/demo/material/expansion_panels_demo.dart';
import 'package:flutter_test/flutter_test.dart';

Future<void> main() async {
  testWidgets('Expansion panel demo: radio tile selection changes on tap', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const MaterialApp(home: ExpansionPanelsDemo()));

    expect(_expandIcons, findsNWidgets(3));

    await tester.tap(find.byWidget(_radioPanelExpandIcon));
    await tester.pumpAndSettle();

    expect(_radioFinder, findsNWidgets(3));

    const int i = 1;

    expect(_isRadioSelected(0), isTrue);
    expect(_isRadioSelected(i), isFalse);

    await tester.tap(find.byWidget(_radioListTiles[i]));
    await tester.pumpAndSettle();

    expect(_isRadioSelected(0), isFalse);
    expect(_isRadioSelected(i), isTrue);
  });
}

Finder get _expandIcons => find.byType(ExpandIcon);

Widget get _radioPanelExpandIcon => _expandIcons.evaluate().toList()[1].widget;

bool _isRadioSelected(int index) => _radios[index].value == _radios[index].groupValue;

List<Radio<Location>> get _radios =>
    List<Radio<Location>>.from(_radioFinder.evaluate().map<Widget>((Element e) => e.widget));

// [find.byType] and [find.widgetWithText] do not match subclasses; `Radio` is
// not sufficient to find a `Radio<_Location>`. Another approach is to grab the
// `runtimeType` of a dummy instance; see
// packages/flutter/test/material/radio_list_tile_test.dart.
Finder get _radioFinder => find.byWidgetPredicate((Widget w) => w is Radio<Location>);

List<RadioListTile<Location>> get _radioListTiles => List<RadioListTile<Location>>.from(
  _radioListTilesFinder.evaluate().map<Widget>((Element e) => e.widget),
);

Finder get _radioListTilesFinder =>
    find.byWidgetPredicate((Widget w) => w is RadioListTile<Location>);
