// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery_localizations.dart';
import 'formatters.dart';

/// Calculates the sum of the primary amounts of a list of [AccountData].
double sumAccountDataPrimaryAmount(List<AccountData> items) =>
    sumOf<AccountData>(items, (AccountData item) => item.primaryAmount);

/// Calculates the sum of the primary amounts of a list of [BillData].
double sumBillDataPrimaryAmount(List<BillData> items) =>
    sumOf<BillData>(items, (BillData item) => item.primaryAmount);

/// Calculates the sum of the primary amounts of a list of [BillData].
double sumBillDataPaidAmount(List<BillData> items) => sumOf<BillData>(
  items.where((BillData item) => item.isPaid).toList(),
  (BillData item) => item.primaryAmount,
);

/// Calculates the sum of the primary amounts of a list of [BudgetData].
double sumBudgetDataPrimaryAmount(List<BudgetData> items) =>
    sumOf<BudgetData>(items, (BudgetData item) => item.primaryAmount);

/// Calculates the sum of the amounts used of a list of [BudgetData].
double sumBudgetDataAmountUsed(List<BudgetData> items) =>
    sumOf<BudgetData>(items, (BudgetData item) => item.amountUsed);

/// Utility function to sum up values in a list.
double sumOf<T>(List<T> list, double Function(T elt) getValue) {
  double sum = 0.0;
  for (final T elt in list) {
    sum += getValue(elt);
  }
  return sum;
}

/// A data model for an account.
///
/// The [primaryAmount] is the balance of the account in USD.
class AccountData {
  const AccountData({required this.name, required this.primaryAmount, required this.accountNumber});

  /// The display name of this entity.
  final String name;

  /// The primary amount or value of this entity.
  final double primaryAmount;

  /// The full displayable account number.
  final String accountNumber;
}

/// A data model for a bill.
///
/// The [primaryAmount] is the amount due in USD.
class BillData {
  const BillData({
    required this.name,
    required this.primaryAmount,
    required this.dueDate,
    this.isPaid = false,
  });

  /// The display name of this entity.
  final String name;

  /// The primary amount or value of this entity.
  final double primaryAmount;

  /// The due date of this bill.
  final String dueDate;

  /// If this bill has been paid.
  final bool isPaid;
}

/// A data model for a budget.
///
/// The [primaryAmount] is the budget cap in USD.
class BudgetData {
  const BudgetData({required this.name, required this.primaryAmount, required this.amountUsed});

  /// The display name of this entity.
  final String name;

  /// The primary amount or value of this entity.
  final double primaryAmount;

  /// Amount of the budget that is consumed or used.
  final double amountUsed;
}

/// A data model for an alert.
class AlertData {
  AlertData({this.message, this.iconData});

  /// The alert message to display.
  final String? message;

  /// The icon to display with the alert.
  final IconData? iconData;
}

class DetailedEventData {
  const DetailedEventData({required this.title, required this.date, required this.amount});

  final String title;
  final DateTime date;
  final double amount;
}

/// A data model for data displayed to the user.
class UserDetailData {
  UserDetailData({required this.title, required this.value});

  /// The display name of this entity.
  final String title;

  /// The value of this entity.
  final String value;
}

/// Class to return dummy data lists.
///
/// In a real app, this might be replaced with some asynchronous service.
class DummyDataService {
  static List<AccountData> getAccountDataList(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <AccountData>[
      AccountData(
        name: localizations.rallyAccountDataChecking,
        primaryAmount: 2215.13,
        accountNumber: '1234561234',
      ),
      AccountData(
        name: localizations.rallyAccountDataHomeSavings,
        primaryAmount: 8678.88,
        accountNumber: '8888885678',
      ),
      AccountData(
        name: localizations.rallyAccountDataCarSavings,
        primaryAmount: 987.48,
        accountNumber: '8888889012',
      ),
      AccountData(
        name: localizations.rallyAccountDataVacation,
        primaryAmount: 253,
        accountNumber: '1231233456',
      ),
    ];
  }

  static List<UserDetailData> getAccountDetailList(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <UserDetailData>[
      UserDetailData(
        title: localizations.rallyAccountDetailDataAnnualPercentageYield,
        value: percentFormat(context).format(0.001),
      ),
      UserDetailData(
        title: localizations.rallyAccountDetailDataInterestRate,
        value: usdWithSignFormat(context).format(1676.14),
      ),
      UserDetailData(
        title: localizations.rallyAccountDetailDataInterestYtd,
        value: usdWithSignFormat(context).format(81.45),
      ),
      UserDetailData(
        title: localizations.rallyAccountDetailDataInterestPaidLastYear,
        value: usdWithSignFormat(context).format(987.12),
      ),
      UserDetailData(
        title: localizations.rallyAccountDetailDataNextStatement,
        value: shortDateFormat(context).format(DateTime.utc(2019, 12, 25)),
      ),
      UserDetailData(title: localizations.rallyAccountDetailDataAccountOwner, value: 'Philip Cao'),
    ];
  }

  static List<DetailedEventData> getDetailedEventItems() {
    // The following titles are not localized as they're product/brand names.
    return <DetailedEventData>[
      DetailedEventData(title: 'Genoe', date: DateTime.utc(2019, 1, 24), amount: -16.54),
      DetailedEventData(
        title: 'Fortnightly Subscribe',
        date: DateTime.utc(2019, 1, 5),
        amount: -12.54,
      ),
      DetailedEventData(title: 'Circle Cash', date: DateTime.utc(2019, 1, 5), amount: 365.65),
      DetailedEventData(
        title: 'Crane Hospitality',
        date: DateTime.utc(2019, 1, 4),
        amount: -705.13,
      ),
      DetailedEventData(title: 'ABC Payroll', date: DateTime.utc(2018, 12, 15), amount: 1141.43),
      DetailedEventData(title: 'Shrine', date: DateTime.utc(2018, 12, 15), amount: -88.88),
      DetailedEventData(title: 'Foodmates', date: DateTime.utc(2018, 12, 4), amount: -11.69),
    ];
  }

  static List<BillData> getBillDataList(BuildContext context) {
    // The following names are not localized as they're product/brand names.
    return <BillData>[
      BillData(
        name: 'RedPay Credit',
        primaryAmount: 45.36,
        dueDate: dateFormatAbbreviatedMonthDay(context).format(DateTime.utc(2019, 1, 29)),
      ),
      BillData(
        name: 'Rent',
        primaryAmount: 1200,
        dueDate: dateFormatAbbreviatedMonthDay(context).format(DateTime.utc(2019, 2, 9)),
        isPaid: true,
      ),
      BillData(
        name: 'TabFine Credit',
        primaryAmount: 87.33,
        dueDate: dateFormatAbbreviatedMonthDay(context).format(DateTime.utc(2019, 2, 22)),
      ),
      BillData(
        name: 'ABC Loans',
        primaryAmount: 400,
        dueDate: dateFormatAbbreviatedMonthDay(context).format(DateTime.utc(2019, 2, 29)),
      ),
    ];
  }

  static List<UserDetailData> getBillDetailList(
    BuildContext context, {
    required double dueTotal,
    required double paidTotal,
  }) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <UserDetailData>[
      UserDetailData(
        title: localizations.rallyBillDetailTotalAmount,
        value: usdWithSignFormat(context).format(paidTotal + dueTotal),
      ),
      UserDetailData(
        title: localizations.rallyBillDetailAmountPaid,
        value: usdWithSignFormat(context).format(paidTotal),
      ),
      UserDetailData(
        title: localizations.rallyBillDetailAmountDue,
        value: usdWithSignFormat(context).format(dueTotal),
      ),
    ];
  }

  static List<BudgetData> getBudgetDataList(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <BudgetData>[
      BudgetData(
        name: localizations.rallyBudgetCategoryCoffeeShops,
        primaryAmount: 70,
        amountUsed: 45.49,
      ),
      BudgetData(
        name: localizations.rallyBudgetCategoryGroceries,
        primaryAmount: 170,
        amountUsed: 16.45,
      ),
      BudgetData(
        name: localizations.rallyBudgetCategoryRestaurants,
        primaryAmount: 170,
        amountUsed: 123.25,
      ),
      BudgetData(
        name: localizations.rallyBudgetCategoryClothing,
        primaryAmount: 70,
        amountUsed: 19.45,
      ),
    ];
  }

  static List<UserDetailData> getBudgetDetailList(
    BuildContext context, {
    required double capTotal,
    required double usedTotal,
  }) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <UserDetailData>[
      UserDetailData(
        title: localizations.rallyBudgetDetailTotalCap,
        value: usdWithSignFormat(context).format(capTotal),
      ),
      UserDetailData(
        title: localizations.rallyBudgetDetailAmountUsed,
        value: usdWithSignFormat(context).format(usedTotal),
      ),
      UserDetailData(
        title: localizations.rallyBudgetDetailAmountLeft,
        value: usdWithSignFormat(context).format(capTotal - usedTotal),
      ),
    ];
  }

  static List<String> getSettingsTitles(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <String>[
      localizations.rallySettingsManageAccounts,
      localizations.rallySettingsTaxDocuments,
      localizations.rallySettingsPasscodeAndTouchId,
      localizations.rallySettingsNotifications,
      localizations.rallySettingsPersonalInformation,
      localizations.rallySettingsPaperlessSettings,
      localizations.rallySettingsFindAtms,
      localizations.rallySettingsHelp,
      localizations.rallySettingsSignOut,
    ];
  }

  static List<AlertData> getAlerts(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <AlertData>[
      AlertData(
        message: localizations.rallyAlertsMessageHeadsUpShopping(
          percentFormat(context, decimalDigits: 0).format(0.9),
        ),
        iconData: Icons.sort,
      ),
      AlertData(
        message: localizations.rallyAlertsMessageSpentOnRestaurants(
          usdWithSignFormat(context, decimalDigits: 0).format(120),
        ),
        iconData: Icons.sort,
      ),
      AlertData(
        message: localizations.rallyAlertsMessageATMFees(
          usdWithSignFormat(context, decimalDigits: 0).format(24),
        ),
        iconData: Icons.credit_card,
      ),
      AlertData(
        message: localizations.rallyAlertsMessageCheckingAccount(
          percentFormat(context, decimalDigits: 0).format(0.04),
        ),
        iconData: Icons.attach_money,
      ),
      AlertData(
        message: localizations.rallyAlertsMessageUnassignedTransactions(16),
        iconData: Icons.not_interested,
      ),
    ];
  }
}
