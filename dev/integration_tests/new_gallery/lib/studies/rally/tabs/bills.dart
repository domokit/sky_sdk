// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

import '../../../gallery_localizations.dart';
import '../charts/pie_chart.dart';
import '../data.dart';
import '../finance.dart';
import 'sidebar.dart';

/// A page that shows a summary of bills.
class BillsView extends StatefulWidget {
  const BillsView({super.key});

  @override
  State<BillsView> createState() => _BillsViewState();
}

class _BillsViewState extends State<BillsView> with SingleTickerProviderStateMixin {
  @override
  Widget build(BuildContext context) {
    final List<BillData> items = DummyDataService.getBillDataList(context);
    final double dueTotal = sumBillDataPrimaryAmount(items);
    final double paidTotal = sumBillDataPaidAmount(items);
    final List<UserDetailData> detailItems = DummyDataService.getBillDetailList(
      context,
      dueTotal: dueTotal,
      paidTotal: paidTotal,
    );

    return TabWithSidebar(
      restorationId: 'bills_view',
      mainView: FinancialEntityView(
        heroLabel: GalleryLocalizations.of(context)!.rallyBillsDue,
        heroAmount: dueTotal,
        segments: buildSegmentsFromBillItems(items),
        wholeAmount: dueTotal,
        financialEntityCards: buildBillDataListViews(items, context),
      ),
      sidebarItems: <Widget>[
        for (final UserDetailData item in detailItems)
          SidebarItem(title: item.title, value: item.value),
      ],
    );
  }
}
