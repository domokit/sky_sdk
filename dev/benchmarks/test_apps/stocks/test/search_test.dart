// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:stocks/main.dart' as stocks;
import 'package:stocks/stock_data.dart' as stock_data;

void main() {
  stock_data.StockData.actuallyFetchData = false;

  testWidgets('Search', (WidgetTester tester) async {
    stocks.main(); // builds the app and schedules a frame but doesn't trigger one
    await tester.pump(); // see https://github.com/flutter/flutter/issues/1865
    await tester.pump(); // triggers a frame

    expect(find.text('AAPL'), findsNothing);
    expect(find.text('BANA'), findsNothing);

    final stocks.StocksAppState app = tester.state<stocks.StocksAppState>(
      find.byType(stocks.StocksApp),
    );
    app.stocks.add(<List<String>>[
      // "Symbol","Name","LastSale","MarketCap","IPOyear","Sector","industry","Summary Quote"
      <String>['AAPL', 'Apple', '', '', '', '', '', ''],
      <String>['BANA', 'Banana', '', '', '', '', '', ''],
    ]);
    await tester.pump();

    expect(find.text('AAPL'), findsOneWidget);
    expect(find.text('BANA'), findsOneWidget);

    await tester.tap(find.byTooltip('Search'));
    // We skip a minute at a time so that each phase of the animation
    // is done in two frames, the start frame and the end frame.
    // There are two phases currently, so that results in three frames.
    expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 3);

    expect(find.text('AAPL'), findsOneWidget);
    expect(find.text('BANA'), findsOneWidget);

    await tester.enterText(find.byType(EditableText), 'B');
    await tester.pump();

    expect(find.text('AAPL'), findsNothing);
    expect(find.text('BANA'), findsOneWidget);

    await tester.enterText(find.byType(EditableText), 'X');
    await tester.pump();

    expect(find.text('AAPL'), findsNothing);
    expect(find.text('BANA'), findsNothing);
  });
}
