// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Kurdish Sorani translations test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        locale: const Locale('ckb'),
        supportedLocales: const <Locale>[Locale('ckb')],
        localizationsDelegates: GlobalMaterialLocalizations.delegates,
        home: Builder(
          builder: (BuildContext context) {
            return Column(
              children: <Widget>[
                Text(MaterialLocalizations.of(context).okButtonLabel),
                Text(MaterialLocalizations.of(context).backButtonTooltip),
              ],
            );
          },
        ),
      ),
    );

    await tester.pumpAndSettle();

    final BuildContext context = tester.element(find.byType(Column));
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);

    expect(localizations.okButtonLabel, 'باشە');
    expect(localizations.backButtonTooltip, 'گەڕانەوە');
    expect(localizations.closeButtonTooltip, 'داخستن');
    expect(localizations.nextMonthTooltip, 'مانگی داهاتوو');
    expect(localizations.previousMonthTooltip, 'مانگی پێشوو');
    expect(localizations.nextPageTooltip, 'لاپەڕەی داهاتوو');
    expect(localizations.previousPageTooltip, 'لاپەڕەی پێشوو');
    expect(localizations.showMenuTooltip, 'پیشاندانی مێنیو');
    expect(localizations.searchFieldLabel, 'گەڕان');
    expect(localizations.cancelButtonLabel, 'هەڵوەشاندنەوە');
    expect(localizations.closeButtonLabel, 'داخستن');
    expect(localizations.continueButtonLabel, 'بەردەوامبوون');
    expect(localizations.copyButtonLabel, 'لەبەرگرتنەوە');
    expect(localizations.cutButtonLabel, 'بڕین');
    expect(localizations.pasteButtonLabel, 'لکاندن');
    expect(localizations.selectAllButtonLabel, 'هەموو هەڵبژێرە');
    expect(localizations.viewLicensesButtonLabel, 'بینینی مۆڵەتەکان');
    expect(localizations.anteMeridiemAbbreviation, 'پ.ن');
    expect(localizations.postMeridiemAbbreviation, 'د.ن');
    expect(localizations.timePickerHourModeAnnouncement, 'هەڵبژاردنی کاتژمێر');
    expect(localizations.timePickerMinuteModeAnnouncement, 'هەڵبژاردنی خولەک');
    expect(localizations.modalBarrierDismissLabel, 'داخستن');
    expect(localizations.drawerLabel, 'مێنیوی ڕێنیشاندەر');
    expect(localizations.popupMenuLabel, 'مێنیوی دەرکەوتوو');
    expect(localizations.dialogLabel, 'دیالۆگ');
    expect(localizations.alertDialogLabel, 'ئاگادارکردنەوە');
    expect(localizations.reorderItemToStart, 'گواستنەوە بۆ سەرەتا');
    expect(localizations.reorderItemToEnd, 'گواستنەوە بۆ کۆتایی');
    expect(localizations.reorderItemUp, 'گواستنەوە بۆ سەرەوە');
    expect(localizations.reorderItemDown, 'گواستنەوە بۆ خوارەوە');
    expect(localizations.reorderItemLeft, 'گواستنەوە بۆ چەپ');
    expect(localizations.reorderItemRight, 'گواستنەوە بۆ ڕاست');
    expect(localizations.expandedIconTapHint, 'داخستنەوە');
    expect(localizations.collapsedIconTapHint, 'فراوانکردن');
    expect(localizations.refreshIndicatorSemanticLabel, 'نوێکردنەوە');
    expect(localizations.moreButtonTooltip, 'زیاتر');
    expect(localizations.dateSeparator, '/');
    expect(localizations.dateHelpText, 'ڕڕ/م/س');
    expect(localizations.selectYearSemanticsLabel, 'هەڵبژاردنی ساڵ');
    expect(localizations.unspecifiedDate, 'بەروار');
    expect(localizations.unspecifiedDateRange, 'ماوەی بەروار');
    expect(localizations.dateInputLabel, 'بەروار بنووسە');
    expect(localizations.dateRangeStartLabel, 'بەرواری دەستپێک');
    expect(localizations.dateRangeEndLabel, 'بەرواری کۆتایی');
    expect(localizations.invalidDateFormatLabel, 'فۆرماتی نادروست.');
    expect(localizations.invalidDateRangeLabel, 'ماوەی نادروست.');
    expect(localizations.dateOutOfRangeLabel, 'دەرەوەی مەودا.');
    expect(localizations.saveButtonLabel, 'پاشەکەوتکردن');
    expect(localizations.datePickerHelpText, 'هەڵبژاردنی بەروار');
    expect(localizations.dateRangePickerHelpText, 'هەڵبژاردنی ماوە');
    expect(localizations.calendarModeButtonLabel, 'گۆڕین بۆ ڕۆژژمێر');
    expect(localizations.inputDateModeButtonLabel, 'گۆڕین بۆ نووسین');
    expect(localizations.timePickerDialHelpText, 'هەڵبژاردنی کات');
    expect(localizations.timePickerInputHelpText, 'نووسینی کات');
    expect(localizations.timePickerHourLabel, 'کاتژمێر');
    expect(localizations.timePickerMinuteLabel, 'خولەک');
    expect(localizations.invalidTimeLabel, 'کاتێکی دروست بنووسە');
    expect(localizations.dialModeButtonLabel, 'گۆڕین بۆ دۆخی هەڵبژێری کاتژمێر');
    expect(localizations.inputTimeModeButtonLabel, 'گۆڕین بۆ دۆخی نووسین');
    expect(localizations.signedInLabel, 'چوونە ژوورەوە');
    expect(localizations.hideAccountsLabel, 'شاردنەوەی هەژمارەکان');
    expect(localizations.showAccountsLabel, 'پیشاندانی هەژمارەکان');
    expect(localizations.menuBarMenuLabel, 'مێنیوی شریتی مێنیو');
    expect(localizations.lookUpButtonLabel, 'گەڕان');
    expect(localizations.searchWebButtonLabel, 'گەڕان لە وێب');
    expect(localizations.shareButtonLabel, 'هاوبەشکردن');
    expect(localizations.clearButtonTooltip, 'سڕینەوەی دەق');
    expect(localizations.selectedDateLabel, 'هەڵبژێردراو');
    expect(localizations.scrimLabel, 'Scrim');
    expect(localizations.scrimOnTapHint('test'), 'داخستنی test');
    expect(localizations.expansionTileExpandedHint, 'دووجار کرتە بکە بۆ داخستنەوە');
    expect(localizations.expansionTileCollapsedHint, 'دووجار کرتە بکە بۆ فراوانکردن');
    expect(localizations.expansionTileExpandedTapHint, 'داخستنەوە');
    expect(localizations.expansionTileCollapsedTapHint, 'فراوانکردن بۆ وردەکاری زیاتر');
    expect(localizations.expandedHint, 'داخراوەتەوە');
    expect(localizations.collapsedHint, 'فراوانکراوە');
    expect(localizations.scanTextButtonLabel, 'سکانکردنی دەق');
  });
}
