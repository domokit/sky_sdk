// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

import '../../../gallery_localizations.dart';
import '../charts/pie_chart.dart';
import '../data.dart';
import '../finance.dart';
import 'sidebar.dart';

class BudgetsView extends StatefulWidget {
  const BudgetsView({super.key});

  @override
  State<BudgetsView> createState() => _BudgetsViewState();
}

class _BudgetsViewState extends State<BudgetsView> with SingleTickerProviderStateMixin {
  @override
  Widget build(BuildContext context) {
    final List<BudgetData> items = DummyDataService.getBudgetDataList(context);
    final double capTotal = sumBudgetDataPrimaryAmount(items);
    final double usedTotal = sumBudgetDataAmountUsed(items);
    final List<UserDetailData> detailItems = DummyDataService.getBudgetDetailList(
      context,
      capTotal: capTotal,
      usedTotal: usedTotal,
    );

    return TabWithSidebar(
      restorationId: 'budgets_view',
      mainView: FinancialEntityView(
        heroLabel: GalleryLocalizations.of(context)!.rallyBudgetLeft,
        heroAmount: capTotal - usedTotal,
        segments: buildSegmentsFromBudgetItems(items),
        wholeAmount: capTotal,
        financialEntityCards: buildBudgetDataListViews(items, context),
      ),
      sidebarItems: <Widget>[
        for (final UserDetailData item in detailItems)
          SidebarItem(title: item.title, value: item.value),
      ],
    );
  }
}
