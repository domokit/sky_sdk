// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Test semantics of TabPageSelector in pt-BR', (WidgetTester tester) async {
    final TabController controller = TabController(vsync: const TestVSync(), length: 2);

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('pt', 'BR'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          GlobalMaterialLocalizations.delegate,
          GlobalWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Column(
              children: <Widget>[
                TabBar(
                  controller: controller,
                  indicatorWeight: 30.0,
                  tabs: const <Widget>[Tab(text: 'TAB1'), Tab(text: 'TAB2')],
                ),
                Flexible(
                  child: TabBarView(
                    controller: controller,
                    children: const <Widget>[Text('PAGE1'), Text('PAGE2')],
                  ),
                ),
                Expanded(child: TabPageSelector(controller: controller)),
              ],
            ),
          ),
        ),
      ),
    );

    final SemanticsHandle handle = tester.ensureSemantics();

    expect(
      tester.getSemantics(find.byType(TabPageSelector)),
      matchesSemantics(label: 'Guia 1 de 2'),
    );

    handle.dispose();
  });
}
