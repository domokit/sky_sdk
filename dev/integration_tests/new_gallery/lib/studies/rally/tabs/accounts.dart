// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../../gallery_localizations.dart';
import '../charts/pie_chart.dart';
import '../data.dart';
import '../finance.dart';
import 'sidebar.dart';

/// A page that shows a summary of accounts.
class AccountsView extends StatelessWidget {
  const AccountsView({super.key});

  @override
  Widget build(BuildContext context) {
    final List<AccountData> items = DummyDataService.getAccountDataList(context);
    final List<UserDetailData> detailItems = DummyDataService.getAccountDetailList(context);
    final double balanceTotal = sumAccountDataPrimaryAmount(items);

    return TabWithSidebar(
      restorationId: 'accounts_view',
      mainView: FinancialEntityView(
        heroLabel: GalleryLocalizations.of(context)!.rallyAccountTotal,
        heroAmount: balanceTotal,
        segments: buildSegmentsFromAccountItems(items),
        wholeAmount: balanceTotal,
        financialEntityCards: buildAccountDataListViews(items, context),
      ),
      sidebarItems: <Widget>[
        for (final UserDetailData item in detailItems)
          SidebarItem(title: item.title, value: item.value),
      ],
    );
  }
}
