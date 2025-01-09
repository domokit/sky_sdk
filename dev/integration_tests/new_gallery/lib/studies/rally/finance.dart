// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import '../../layout/adaptive.dart';
import '../../layout/text_scale.dart';
import 'charts/line_chart.dart';
import 'charts/pie_chart.dart';
import 'charts/vertical_fraction_bar.dart';
import 'colors.dart';
import 'data.dart';
import 'formatters.dart';

class FinancialEntityView extends StatelessWidget {
  const FinancialEntityView({
    super.key,
    required this.heroLabel,
    required this.heroAmount,
    required this.wholeAmount,
    required this.segments,
    required this.financialEntityCards,
  }) : assert(segments.length == financialEntityCards.length);

  /// The amounts to assign each item.
  final List<RallyPieChartSegment> segments;
  final String heroLabel;
  final double heroAmount;
  final double wholeAmount;
  final List<FinancialEntityCategoryView> financialEntityCards;

  @override
  Widget build(BuildContext context) {
    final double maxWidth = pieChartMaxSize + (cappedTextScale(context) - 1.0) * 100.0;
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        return Column(
          children: <Widget>[
            ConstrainedBox(
              constraints: BoxConstraints(
                // We decrease the max height to ensure the [RallyPieChart] does
                // not take up the full height when it is smaller than
                // [kPieChartMaxSize].
                maxHeight: math.min(constraints.biggest.shortestSide * 0.9, maxWidth),
              ),
              child: RallyPieChart(
                heroLabel: heroLabel,
                heroAmount: heroAmount,
                wholeAmount: wholeAmount,
                segments: segments,
              ),
            ),
            const SizedBox(height: 24),
            Container(
              height: 1,
              constraints: BoxConstraints(maxWidth: maxWidth),
              color: RallyColors.inputBackground,
            ),
            Container(
              constraints: BoxConstraints(maxWidth: maxWidth),
              color: RallyColors.cardBackground,
              child: Column(children: financialEntityCards),
            ),
          ],
        );
      },
    );
  }
}

/// A reusable widget to show balance information of a single entity as a card.
class FinancialEntityCategoryView extends StatelessWidget {
  const FinancialEntityCategoryView({
    super.key,
    required this.indicatorColor,
    required this.indicatorFraction,
    required this.title,
    required this.subtitle,
    required this.semanticsLabel,
    required this.amount,
    required this.suffix,
  });

  final Color indicatorColor;
  final double indicatorFraction;
  final String title;
  final String subtitle;
  final String semanticsLabel;
  final String amount;
  final Widget suffix;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Semantics.fromProperties(
      properties: SemanticsProperties(button: true, enabled: true, label: semanticsLabel),
      excludeSemantics: true,
      // TODO(x): State restoration of FinancialEntityCategoryDetailsPage on mobile is blocked because OpenContainer does not support restorablePush, https://github.com/flutter/gallery/issues/570.
      child: OpenContainer(
        transitionDuration: const Duration(milliseconds: 350),
        openBuilder:
            (BuildContext context, void Function() openContainer) =>
                FinancialEntityCategoryDetailsPage(),
        openColor: RallyColors.primaryBackground,
        closedColor: RallyColors.primaryBackground,
        closedElevation: 0,
        closedBuilder: (BuildContext context, void Function() openContainer) {
          return TextButton(
            style: TextButton.styleFrom(foregroundColor: Colors.black),
            onPressed: openContainer,
            child: Column(
              children: <Widget>[
                Container(
                  padding: const EdgeInsets.symmetric(vertical: 16, horizontal: 8),
                  child: Row(
                    children: <Widget>[
                      Container(
                        alignment: Alignment.center,
                        height: 32 + 60 * (cappedTextScale(context) - 1),
                        padding: const EdgeInsets.symmetric(horizontal: 12),
                        child: VerticalFractionBar(
                          color: indicatorColor,
                          fraction: indicatorFraction,
                        ),
                      ),
                      Expanded(
                        child: Wrap(
                          alignment: WrapAlignment.spaceBetween,
                          crossAxisAlignment: WrapCrossAlignment.center,
                          children: <Widget>[
                            Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: <Widget>[
                                Text(title, style: textTheme.bodyMedium!.copyWith(fontSize: 16)),
                                Text(
                                  subtitle,
                                  style: textTheme.bodyMedium!.copyWith(color: RallyColors.gray60),
                                ),
                              ],
                            ),
                            Text(
                              amount,
                              style: textTheme.bodyLarge!.copyWith(
                                fontSize: 20,
                                color: RallyColors.gray,
                              ),
                            ),
                          ],
                        ),
                      ),
                      Container(
                        constraints: const BoxConstraints(minWidth: 32),
                        padding: const EdgeInsetsDirectional.only(start: 12),
                        child: suffix,
                      ),
                    ],
                  ),
                ),
                const Divider(
                  height: 1,
                  indent: 16,
                  endIndent: 16,
                  color: RallyColors.dividerColor,
                ),
              ],
            ),
          );
        },
      ),
    );
  }
}

/// Data model for [FinancialEntityCategoryView].
class FinancialEntityCategoryModel {
  const FinancialEntityCategoryModel(
    this.indicatorColor,
    this.indicatorFraction,
    this.title,
    this.subtitle,
    this.usdAmount,
    this.suffix,
  );

  final Color indicatorColor;
  final double indicatorFraction;
  final String title;
  final String subtitle;
  final double usdAmount;
  final Widget suffix;
}

FinancialEntityCategoryView buildFinancialEntityFromAccountData(
  AccountData model,
  int accountDataIndex,
  BuildContext context,
) {
  final String amount = usdWithSignFormat(context).format(model.primaryAmount);
  final String shortAccountNumber = model.accountNumber.substring(6);
  return FinancialEntityCategoryView(
    suffix: const Icon(Icons.chevron_right, color: Colors.grey),
    title: model.name,
    subtitle: '• • • • • • $shortAccountNumber',
    semanticsLabel: GalleryLocalizations.of(
      context,
    )!.rallyAccountAmount(model.name, shortAccountNumber, amount),
    indicatorColor: RallyColors.accountColor(accountDataIndex),
    indicatorFraction: 1,
    amount: amount,
  );
}

FinancialEntityCategoryView buildFinancialEntityFromBillData(
  BillData model,
  int billDataIndex,
  BuildContext context,
) {
  final String amount = usdWithSignFormat(context).format(model.primaryAmount);
  return FinancialEntityCategoryView(
    suffix: const Icon(Icons.chevron_right, color: Colors.grey),
    title: model.name,
    subtitle: model.dueDate,
    semanticsLabel: GalleryLocalizations.of(
      context,
    )!.rallyBillAmount(model.name, model.dueDate, amount),
    indicatorColor: RallyColors.billColor(billDataIndex),
    indicatorFraction: 1,
    amount: amount,
  );
}

FinancialEntityCategoryView buildFinancialEntityFromBudgetData(
  BudgetData model,
  int budgetDataIndex,
  BuildContext context,
) {
  final String amountUsed = usdWithSignFormat(context).format(model.amountUsed);
  final String primaryAmount = usdWithSignFormat(context).format(model.primaryAmount);
  final String amount = usdWithSignFormat(context).format(model.primaryAmount - model.amountUsed);

  return FinancialEntityCategoryView(
    suffix: Text(
      GalleryLocalizations.of(context)!.rallyFinanceLeft,
      style: Theme.of(
        context,
      ).textTheme.bodyMedium!.copyWith(color: RallyColors.gray60, fontSize: 10),
    ),
    title: model.name,
    subtitle: '$amountUsed / $primaryAmount',
    semanticsLabel: GalleryLocalizations.of(
      context,
    )!.rallyBudgetAmount(model.name, model.amountUsed, model.primaryAmount, amount),
    indicatorColor: RallyColors.budgetColor(budgetDataIndex),
    indicatorFraction: model.amountUsed / model.primaryAmount,
    amount: amount,
  );
}

List<FinancialEntityCategoryView> buildAccountDataListViews(
  List<AccountData> items,
  BuildContext context,
) {
  return List<FinancialEntityCategoryView>.generate(
    items.length,
    (int i) => buildFinancialEntityFromAccountData(items[i], i, context),
  );
}

List<FinancialEntityCategoryView> buildBillDataListViews(
  List<BillData> items,
  BuildContext context,
) {
  return List<FinancialEntityCategoryView>.generate(
    items.length,
    (int i) => buildFinancialEntityFromBillData(items[i], i, context),
  );
}

List<FinancialEntityCategoryView> buildBudgetDataListViews(
  List<BudgetData> items,
  BuildContext context,
) {
  return <FinancialEntityCategoryView>[
    for (int i = 0; i < items.length; i++) buildFinancialEntityFromBudgetData(items[i], i, context),
  ];
}

class FinancialEntityCategoryDetailsPage extends StatelessWidget {
  FinancialEntityCategoryDetailsPage({super.key});

  final List<DetailedEventData> items = DummyDataService.getDetailedEventItems();

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);

    return ApplyTextOptions(
      child: Scaffold(
        appBar: AppBar(
          elevation: 0,
          centerTitle: true,
          title: Text(
            GalleryLocalizations.of(context)!.rallyAccountDataChecking,
            style: Theme.of(context).textTheme.bodyMedium!.copyWith(fontSize: 18),
          ),
        ),
        body: Column(
          children: <Widget>[
            SizedBox(height: 200, width: double.infinity, child: RallyLineChart(events: items)),
            Expanded(
              child: Padding(
                padding: isDesktop ? const EdgeInsets.all(40) : EdgeInsets.zero,
                child: ListView(
                  shrinkWrap: true,
                  children: <Widget>[
                    for (final DetailedEventData detailedEventData in items)
                      _DetailedEventCard(
                        title: detailedEventData.title,
                        date: detailedEventData.date,
                        amount: detailedEventData.amount,
                      ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _DetailedEventCard extends StatelessWidget {
  const _DetailedEventCard({required this.title, required this.date, required this.amount});

  final String title;
  final DateTime date;
  final double amount;

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    return TextButton(
      style: TextButton.styleFrom(
        foregroundColor: Colors.black,
        padding: const EdgeInsets.symmetric(horizontal: 16),
      ),
      onPressed: () {},
      child: Column(
        children: <Widget>[
          Container(
            padding: const EdgeInsets.symmetric(vertical: 16),
            width: double.infinity,
            child:
                isDesktop
                    ? Row(
                      children: <Widget>[
                        Expanded(child: _EventTitle(title: title)),
                        _EventDate(date: date),
                        Expanded(
                          child: Align(
                            alignment: AlignmentDirectional.centerEnd,
                            child: _EventAmount(amount: amount),
                          ),
                        ),
                      ],
                    )
                    : Wrap(
                      alignment: WrapAlignment.spaceBetween,
                      children: <Widget>[
                        Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: <Widget>[_EventTitle(title: title), _EventDate(date: date)],
                        ),
                        _EventAmount(amount: amount),
                      ],
                    ),
          ),
          SizedBox(height: 1, child: Container(color: RallyColors.dividerColor)),
        ],
      ),
    );
  }
}

class _EventAmount extends StatelessWidget {
  const _EventAmount({required this.amount});

  final double amount;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Text(
      usdWithSignFormat(context).format(amount),
      style: textTheme.bodyLarge!.copyWith(fontSize: 20, color: RallyColors.gray),
    );
  }
}

class _EventDate extends StatelessWidget {
  const _EventDate({required this.date});

  final DateTime date;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Text(
      shortDateFormat(context).format(date),
      semanticsLabel: longDateFormat(context).format(date),
      style: textTheme.bodyMedium!.copyWith(color: RallyColors.gray60),
    );
  }
}

class _EventTitle extends StatelessWidget {
  const _EventTitle({required this.title});

  final String title;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Text(title, style: textTheme.bodyMedium!.copyWith(fontSize: 16));
  }
}
