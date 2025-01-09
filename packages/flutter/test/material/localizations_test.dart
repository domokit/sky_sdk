// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('English translations exist for all MaterialLocalizations properties', (
    WidgetTester tester,
  ) async {
    const MaterialLocalizations localizations = DefaultMaterialLocalizations();

    expect(localizations.openAppDrawerTooltip, isNotNull);
    expect(localizations.backButtonTooltip, isNotNull);
    expect(localizations.clearButtonTooltip, isNotNull);
    expect(localizations.closeButtonTooltip, isNotNull);
    expect(localizations.deleteButtonTooltip, isNotNull);
    expect(localizations.moreButtonTooltip, isNotNull);
    expect(localizations.nextMonthTooltip, isNotNull);
    expect(localizations.previousMonthTooltip, isNotNull);
    expect(localizations.nextPageTooltip, isNotNull);
    expect(localizations.previousPageTooltip, isNotNull);
    expect(localizations.firstPageTooltip, isNotNull);
    expect(localizations.lastPageTooltip, isNotNull);
    expect(localizations.showMenuTooltip, isNotNull);
    expect(localizations.licensesPageTitle, isNotNull);
    expect(localizations.rowsPerPageTitle, isNotNull);
    expect(localizations.cancelButtonLabel, isNotNull);
    expect(localizations.closeButtonLabel, isNotNull);
    expect(localizations.continueButtonLabel, isNotNull);
    expect(localizations.copyButtonLabel, isNotNull);
    expect(localizations.cutButtonLabel, isNotNull);
    expect(localizations.scanTextButtonLabel, isNotNull);
    expect(localizations.lookUpButtonLabel, isNotNull);
    expect(localizations.searchWebButtonLabel, isNotNull);
    expect(localizations.shareButtonLabel, isNotNull);
    expect(localizations.okButtonLabel, isNotNull);
    expect(localizations.pasteButtonLabel, isNotNull);
    expect(localizations.selectAllButtonLabel, isNotNull);
    expect(localizations.viewLicensesButtonLabel, isNotNull);
    expect(localizations.anteMeridiemAbbreviation, isNotNull);
    expect(localizations.postMeridiemAbbreviation, isNotNull);
    expect(localizations.timePickerHourModeAnnouncement, isNotNull);
    expect(localizations.timePickerMinuteModeAnnouncement, isNotNull);
    expect(localizations.modalBarrierDismissLabel, isNotNull);
    expect(localizations.menuDismissLabel, isNotNull);
    expect(localizations.drawerLabel, isNotNull);
    expect(localizations.menuBarMenuLabel, isNotNull);
    expect(localizations.popupMenuLabel, isNotNull);
    expect(localizations.dialogLabel, isNotNull);
    expect(localizations.alertDialogLabel, isNotNull);
    expect(localizations.searchFieldLabel, isNotNull);
    expect(localizations.dateSeparator, isNotNull);
    expect(localizations.dateHelpText, isNotNull);
    expect(localizations.selectYearSemanticsLabel, isNotNull);
    expect(localizations.unspecifiedDate, isNotNull);
    expect(localizations.unspecifiedDateRange, isNotNull);
    expect(localizations.dateInputLabel, isNotNull);
    expect(localizations.dateRangeStartLabel, isNotNull);
    expect(localizations.dateRangeEndLabel, isNotNull);
    expect(localizations.invalidDateFormatLabel, isNotNull);
    expect(localizations.invalidDateRangeLabel, isNotNull);
    expect(localizations.dateOutOfRangeLabel, isNotNull);
    expect(localizations.saveButtonLabel, isNotNull);
    expect(localizations.datePickerHelpText, isNotNull);
    expect(localizations.dateRangePickerHelpText, isNotNull);
    expect(localizations.calendarModeButtonLabel, isNotNull);
    expect(localizations.inputDateModeButtonLabel, isNotNull);
    expect(localizations.timePickerDialHelpText, isNotNull);
    expect(localizations.timePickerInputHelpText, isNotNull);
    expect(localizations.timePickerHourLabel, isNotNull);
    expect(localizations.timePickerMinuteLabel, isNotNull);
    expect(localizations.invalidTimeLabel, isNotNull);
    expect(localizations.dialModeButtonLabel, isNotNull);
    expect(localizations.inputTimeModeButtonLabel, isNotNull);
    expect(localizations.signedInLabel, isNotNull);
    expect(localizations.hideAccountsLabel, isNotNull);
    expect(localizations.showAccountsLabel, isNotNull);
    expect(localizations.reorderItemToStart, isNotNull);
    expect(localizations.reorderItemToEnd, isNotNull);
    expect(localizations.reorderItemUp, isNotNull);
    expect(localizations.reorderItemDown, isNotNull);
    expect(localizations.reorderItemLeft, isNotNull);
    expect(localizations.reorderItemRight, isNotNull);
    expect(localizations.expandedIconTapHint, isNotNull);
    expect(localizations.collapsedIconTapHint, isNotNull);
    expect(localizations.expansionTileExpandedHint, isNotNull);
    expect(localizations.expansionTileCollapsedHint, isNotNull);
    expect(localizations.expandedHint, isNotNull);
    expect(localizations.collapsedHint, isNotNull);
    expect(localizations.keyboardKeyAlt, isNotNull);
    expect(localizations.keyboardKeyAltGraph, isNotNull);
    expect(localizations.keyboardKeyBackspace, isNotNull);
    expect(localizations.keyboardKeyCapsLock, isNotNull);
    expect(localizations.keyboardKeyChannelDown, isNotNull);
    expect(localizations.keyboardKeyChannelUp, isNotNull);
    expect(localizations.keyboardKeyControl, isNotNull);
    expect(localizations.keyboardKeyDelete, isNotNull);
    expect(localizations.keyboardKeyEject, isNotNull);
    expect(localizations.keyboardKeyEnd, isNotNull);
    expect(localizations.keyboardKeyEscape, isNotNull);
    expect(localizations.keyboardKeyFn, isNotNull);
    expect(localizations.keyboardKeyHome, isNotNull);
    expect(localizations.keyboardKeyInsert, isNotNull);
    expect(localizations.keyboardKeyMeta, isNotNull);
    expect(localizations.keyboardKeyMetaMacOs, isNotNull);
    expect(localizations.keyboardKeyMetaWindows, isNotNull);
    expect(localizations.keyboardKeyNumLock, isNotNull);
    expect(localizations.keyboardKeyNumpad1, isNotNull);
    expect(localizations.keyboardKeyNumpad2, isNotNull);
    expect(localizations.keyboardKeyNumpad3, isNotNull);
    expect(localizations.keyboardKeyNumpad4, isNotNull);
    expect(localizations.keyboardKeyNumpad5, isNotNull);
    expect(localizations.keyboardKeyNumpad6, isNotNull);
    expect(localizations.keyboardKeyNumpad7, isNotNull);
    expect(localizations.keyboardKeyNumpad8, isNotNull);
    expect(localizations.keyboardKeyNumpad9, isNotNull);
    expect(localizations.keyboardKeyNumpad0, isNotNull);
    expect(localizations.keyboardKeyNumpadAdd, isNotNull);
    expect(localizations.keyboardKeyNumpadComma, isNotNull);
    expect(localizations.keyboardKeyNumpadDecimal, isNotNull);
    expect(localizations.keyboardKeyNumpadDivide, isNotNull);
    expect(localizations.keyboardKeyNumpadEnter, isNotNull);
    expect(localizations.keyboardKeyNumpadEqual, isNotNull);
    expect(localizations.keyboardKeyNumpadMultiply, isNotNull);
    expect(localizations.keyboardKeyNumpadParenLeft, isNotNull);
    expect(localizations.keyboardKeyNumpadParenRight, isNotNull);
    expect(localizations.keyboardKeyNumpadSubtract, isNotNull);
    expect(localizations.keyboardKeyPageDown, isNotNull);
    expect(localizations.keyboardKeyPageUp, isNotNull);
    expect(localizations.keyboardKeyPower, isNotNull);
    expect(localizations.keyboardKeyPowerOff, isNotNull);
    expect(localizations.keyboardKeyPrintScreen, isNotNull);
    expect(localizations.keyboardKeyScrollLock, isNotNull);
    expect(localizations.keyboardKeySelect, isNotNull);
    expect(localizations.keyboardKeyShift, isNotNull);
    expect(localizations.keyboardKeySpace, isNotNull);
    expect(localizations.currentDateLabel, isNotNull);
    expect(localizations.scrimLabel, isNotNull);
    expect(localizations.bottomSheetLabel, isNotNull);
    expect(localizations.selectedDateLabel, isNotNull);

    expect(localizations.scrimOnTapHint('FOO'), contains('FOO'));

    expect(localizations.aboutListTileTitle('FOO'), isNotNull);
    expect(localizations.aboutListTileTitle('FOO'), contains('FOO'));

    expect(localizations.selectedRowCountTitle(0), isNotNull);
    expect(localizations.selectedRowCountTitle(1), isNotNull);
    expect(localizations.selectedRowCountTitle(2), isNotNull);
    expect(localizations.selectedRowCountTitle(100), isNotNull);
    expect(localizations.selectedRowCountTitle(0).contains(r'$selectedRowCount'), isFalse);
    expect(localizations.selectedRowCountTitle(1).contains(r'$selectedRowCount'), isFalse);
    expect(localizations.selectedRowCountTitle(2).contains(r'$selectedRowCount'), isFalse);
    expect(localizations.selectedRowCountTitle(100).contains(r'$selectedRowCount'), isFalse);

    expect(localizations.pageRowsInfoTitle(1, 10, 100, true), isNotNull);
    expect(localizations.pageRowsInfoTitle(1, 10, 100, false), isNotNull);
    expect(localizations.pageRowsInfoTitle(1, 10, 100, true).contains(r'$firstRow'), isFalse);
    expect(localizations.pageRowsInfoTitle(1, 10, 100, true).contains(r'$lastRow'), isFalse);
    expect(localizations.pageRowsInfoTitle(1, 10, 100, true).contains(r'$rowCount'), isFalse);
    expect(localizations.pageRowsInfoTitle(1, 10, 100, false).contains(r'$firstRow'), isFalse);
    expect(localizations.pageRowsInfoTitle(1, 10, 100, false).contains(r'$lastRow'), isFalse);
    expect(localizations.pageRowsInfoTitle(1, 10, 100, false).contains(r'$rowCount'), isFalse);

    expect(localizations.licensesPackageDetailText(0), isNotNull);
    expect(localizations.licensesPackageDetailText(1), isNotNull);
    expect(localizations.licensesPackageDetailText(2), isNotNull);
    expect(localizations.licensesPackageDetailText(100), isNotNull);
    expect(localizations.licensesPackageDetailText(1).contains(r'$licensesCount'), isFalse);
    expect(localizations.licensesPackageDetailText(2).contains(r'$licensesCount'), isFalse);
    expect(localizations.licensesPackageDetailText(100).contains(r'$licensesCount'), isFalse);
  });

  testWidgets('MaterialLocalizations.of throws', (WidgetTester tester) async {
    final GlobalKey noLocalizationsAvailable = GlobalKey();
    final GlobalKey localizationsAvailable = GlobalKey();

    await tester.pumpWidget(
      Container(
        key: noLocalizationsAvailable,
        child: MaterialApp(home: Container(key: localizationsAvailable)),
      ),
    );

    expect(
      () => MaterialLocalizations.of(noLocalizationsAvailable.currentContext!),
      throwsA(
        isAssertionError.having(
          (AssertionError e) => e.message,
          'message',
          contains('No MaterialLocalizations found'),
        ),
      ),
    );

    expect(
      MaterialLocalizations.of(localizationsAvailable.currentContext!),
      isA<MaterialLocalizations>(),
    );
  });

  testWidgets("parseCompactDate doesn't throw an exception on invalid text", (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/126397.
    final GlobalKey localizations = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(home: Material(key: localizations, child: const SizedBox.expand())),
    );

    final MaterialLocalizations materialLocalizations = MaterialLocalizations.of(
      localizations.currentContext!,
    );
    expect(materialLocalizations.parseCompactDate('10/05/2023'), isNotNull);
    expect(tester.takeException(), null);

    expect(materialLocalizations.parseCompactDate('10/05/2023666777889'), null);
    expect(tester.takeException(), null);
  });
}
