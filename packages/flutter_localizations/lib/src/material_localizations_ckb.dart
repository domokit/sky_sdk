// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:intl/intl.dart' as intl;

import 'material_localizations.dart';

/// Material Localizations for Kurdish (Sorani).
///
/// See also:
///
///  * [GlobalMaterialLocalizations], which provides material localizations for
///    many languages.
class MaterialLocalizationCkb extends GlobalMaterialLocalizations {
  /// Create an instance of the translation bundle for Kurdish (Sorani).
  const MaterialLocalizationCkb({
    String localeName = 'ckb',
    required super.fullYearFormat,
    required super.compactDateFormat,
    required super.shortDateFormat,
    required super.mediumDateFormat,
    required super.longDateFormat,
    required super.yearMonthFormat,
    required super.shortMonthDayFormat,
    required super.decimalFormat,
    required super.twoDigitZeroPaddedFormat,
  }) : super(localeName: localeName);

  @override
  TextDirection get textDirection => TextDirection.rtl;

  @override
  String get aboutListTileTitleRaw => r'دەربارەی $applicationName';

  @override
  String get alertDialogLabel => 'ئاگادارکردنەوە';

  @override
  String get anteMeridiemAbbreviation => 'پ.ن';

  @override
  String get backButtonTooltip => 'گەڕانەوە';

  @override
  String get bottomSheetLabel => 'پەڕەی خوارەوە';

  @override
  String get calendarModeButtonLabel => 'گۆڕین بۆ ڕۆژژمێر';

  @override
  String get cancelButtonLabel => 'هەڵوەشاندنەوە';

  @override
  String get closeButtonLabel => 'داخستن';

  @override
  String get closeButtonTooltip => 'داخستن';

  @override
  String get collapsedIconTapHint => 'فراوانکردن';

  @override
  String get continueButtonLabel => 'بەردەوامبوون';

  @override
  String get copyButtonLabel => 'لەبەرگرتنەوە';

  @override
  String get currentDateLabel => 'ئەمڕۆ';

  @override
  String get cutButtonLabel => 'بڕین';

  @override
  String get dateHelpText => 'ڕڕ/م/س';

  @override
  String get dateInputLabel => 'بەروار بنووسە';

  @override
  String get dateOutOfRangeLabel => 'دەرەوەی مەودا.';

  @override
  String get datePickerHelpText => 'هەڵبژا��دنی بەروار';

  @override
  String get dateRangeEndDateSemanticLabelRaw => r'بەرواری کۆتایی $fullDate';

  @override
  String get dateRangeEndLabel => 'بەرواری کۆتایی';

  @override
  String get dateRangePickerHelpText => 'هەڵبژاردنی ماوە';

  @override
  String get dateRangeStartDateSemanticLabelRaw => r'بەرواری دەستپێک $fullDate';

  @override
  String get dateRangeStartLabel => 'بەرواری دەستپێک';

  @override
  String get dateSeparator => '/';

  @override
  String get deleteButtonTooltip => 'سڕینەوە';

  @override
  String get dialModeButtonLabel => 'گۆڕین بۆ دۆخی هەڵبژێری کاتژمێر';

  @override
  String get dialogLabel => 'دیالۆگ';

  @override
  String get drawerLabel => 'مێنیوی ڕێنیشاندەر';

  @override
  String get expandedIconTapHint => 'داخستنەوە';

  @override
  String get firstPageTooltip => 'یەکەم لاپەڕە';

  @override
  String get hideAccountsLabel => 'شاردنەوەی هەژمارەکان';

  @override
  String get inputDateModeButtonLabel => 'گۆڕین بۆ نووسین';

  @override
  String get inputTimeModeButtonLabel => 'گۆڕین بۆ دۆخی نووسین';

  @override
  String get invalidDateFormatLabel => 'فۆرماتی ناد��وست.';

  @override
  String get invalidDateRangeLabel => 'ماوەی نادروست.';

  @override
  String get invalidTimeLabel => 'کاتێکی دروست بنووسە';

  @override
  String get lastPageTooltip => 'دوا لاپەڕە';

  @override
  String get licensesPackageDetailTextOne => '١ مۆڵەت';

  @override
  String get licensesPackageDetailTextOther => r'$licenseCount مۆڵەت';

  @override
  String get licensesPackageDetailTextZero => 'هیچ مۆڵەتێک نییە';

  @override
  String get licensesPageTitle => 'مۆڵەتەکان';

  @override
  String get modalBarrierDismissLabel => 'دەرچوون';

  @override
  String get moreButtonTooltip => 'زیاتر';

  @override
  String get nextMonthTooltip => 'مانگی داهاتوو';

  @override
  String get nextPageTooltip => 'لاپەڕەی داهاتوو';

  @override
  String get okButtonLabel => 'باشە';

  @override
  String get openAppDrawerTooltip => 'کردنەوەی مێنیوی ڕێنیشاندەر';

  @override
  String get pageRowsInfoTitleRaw => r'$firstRow–$lastRow لە $rowCount';

  @override
  String get pageRowsInfoTitleApproximateRaw => r'$firstRow–$lastRow لە نزیکەی $rowCount';

  @override
  String get pasteButtonLabel => 'لکاندن';

  @override
  String get popupMenuLabel => 'مێنیوی دەرکەوتوو';

  @override
  String get postMeridiemAbbreviation => 'د.ن';

  @override
  String get previousMonthTooltip => 'مانگی پێشوو';

  @override
  String get previousPageTooltip => 'لاپەڕەی پێشوو';

  @override
  String get refreshIndicatorSemanticLabel => 'نوێکردنەوە';

  @override
  String get remainingTextFieldCharacterCountOne => '١ پیت ماوە';

  @override
  String get remainingTextFieldCharacterCountOther => r'$remainingCount پیت ماوە';

  @override
  String get remainingTextFieldCharacterCountZero => 'هیچ پیتێک نەماوە';

  @override
  String get reorderItemDown => 'گواستنەوە بۆ خوارەوە';

  @override
  String get reorderItemLeft => 'گواستنەوە بۆ چەپ';

  @override
  String get reorderItemRight => 'گواستنەوە بۆ ڕاست';

  @override
  String get reorderItemToEnd => 'گواستنەوە بۆ کۆتایی';

  @override
  String get reorderItemToStart => 'گواستنەوە بۆ سەرەتا';

  @override
  String get reorderItemUp => 'گواستنەوە بۆ سەرەوە';

  @override
  String get rowsPerPageTitle => 'ڕیز لە هەر لاپەڕەیەک:';

  @override
  String get saveButtonLabel => 'پاشەکەوتکردن';

  @override
  String get searchFieldLabel => 'گەڕان';

  @override
  String get selectAllButtonLabel => 'هەڵبژاردنی هەموو';

  @override
  String get selectYearSemanticsLabel => 'هەڵبژاردنی ساڵ';

  @override
  String get selectedRowCountTitleOne => '١ دانە هەڵبژێردرا';

  @override
  String get selectedRowCountTitleOther => r'$selectedRowCount دانە هەڵبژێردران';

  @override
  String get selectedRowCountTitleZero => 'هیچ هەڵنەبژێردراوە';

  @override
  String get showAccountsLabel => 'پیشاندانی هەژمارەکان';

  @override
  String get showMenuTooltip => 'پیشاندانی مێنیو';

  @override
  String get signedInLabel => 'چوونە ژوورەوە';

  @override
  String get tabLabelRaw => r'تابی $tabIndex لە $tabCount';

  @override
  String get timePickerDialHelpText => 'هەڵبژاردنی کات';

  @override
  String get timePickerHourLabel => 'کاتژمێر';

  @override
  String get timePickerHourModeAnnouncement => 'هەڵبژاردنی کاتژمێر';

  @override
  String get timePickerInputHelpText => 'نووسینی کات';

  @override
  String get timePickerMinuteLabel => 'خولەک';

  @override
  String get timePickerMinuteModeAnnouncement => 'هەڵبژاردنی خولەک';

  @override
  String get unspecifiedDate => 'بەروار';

  @override
  String get unspecifiedDateRange => 'ماوەی بەروار';

  @override
  String get viewLicensesButtonLabel => 'بینینی مۆڵەتەکان';

  @override
  String get lookUpButtonLabel => 'گەڕان';

  @override
  String get menuDismissLabel => 'داخستنی مێنیو';

  @override
  String get searchWebButtonLabel => 'گەڕان لە وێب';

  @override
  String get shareButtonLabel => 'هاوبەشکردن';

  @override
  String get clearButtonTooltip => 'سڕینەوەی دەق';

  @override
  String get selectedDateLabel => 'هەڵبژێردراو';

  @override
  String get scrimLabel => 'Scrim';

  @override
  String get scrimOnTapHintRaw => r'داخستنی $modalRouteContentName';

  @override
  String get expansionTileExpandedHint => 'دووجار کرتە بکە بۆ داخستنەوە';

  @override
  String get expansionTileCollapsedHint => 'دووجار کرتە بکە بۆ فراوانکردن';

  @override
  String get expansionTileExpandedTapHint => 'داخستنەوە';

  @override
  String get expansionTileCollapsedTapHint => 'فراوانکردن بۆ وردەکاری زیاتر';

  @override
  String get expandedHint => 'داخراوەتەوە';

  @override
  String get collapsedHint => 'فراوانکراوە';

  @override
  String get menuBarMenuLabel => 'مێنیوی شریتی مێنیو';

  @override
  String get scanTextButtonLabel => 'سکانکردنی دەق';

  @override
  ScriptCategory get scriptCategory => ScriptCategory.tall;

  @override
  TimeOfDayFormat get timeOfDayFormatRaw => TimeOfDayFormat.H_colon_mm;

  @override
  String get keyboardKeyAlt => 'Alt';

  @override
  String get keyboardKeyAltGraph => 'AltGr';

  @override
  String get keyboardKeyBackspace => 'Backspace';

  @override
  String get keyboardKeyCapsLock => 'Caps Lock';

  @override
  String get keyboardKeyChannelDown => 'کەناڵی خوارەوە';

  @override
  String get keyboardKeyChannelUp => 'کەناڵی سەرەوە';

  @override
  String get keyboardKeyControl => 'Ctrl';

  @override
  String get keyboardKeyDelete => 'Del';

  @override
  String get keyboardKeyEject => 'Eject';

  @override
  String get keyboardKeyEnd => 'End';

  @override
  String get keyboardKeyEscape => 'Esc';

  @override
  String get keyboardKeyFn => 'Fn';

  @override
  String get keyboardKeyHome => 'Home';

  @override
  String get keyboardKeyInsert => 'Insert';

  @override
  String get keyboardKeyMeta => 'Meta';

  @override
  String get keyboardKeyMetaMacOs => 'Command';

  @override
  String get keyboardKeyMetaWindows => 'Win';

  @override
  String get keyboardKeyNumLock => 'Num Lock';

  @override
  String get keyboardKeyNumpad0 => 'Num 0';

  @override
  String get keyboardKeyNumpad1 => 'Num 1';

  @override
  String get keyboardKeyNumpad2 => 'Num 2';

  @override
  String get keyboardKeyNumpad3 => 'Num 3';

  @override
  String get keyboardKeyNumpad4 => 'Num 4';

  @override
  String get keyboardKeyNumpad5 => 'Num 5';

  @override
  String get keyboardKeyNumpad6 => 'Num 6';

  @override
  String get keyboardKeyNumpad7 => 'Num 7';

  @override
  String get keyboardKeyNumpad8 => 'Num 8';

  @override
  String get keyboardKeyNumpad9 => 'Num 9';

  @override
  String get keyboardKeyNumpadAdd => 'Num +';

  @override
  String get keyboardKeyNumpadComma => 'Num ,';

  @override
  String get keyboardKeyNumpadDecimal => 'Num .';

  @override
  String get keyboardKeyNumpadDivide => 'Num /';

  @override
  String get keyboardKeyNumpadEnter => 'Num Enter';

  @override
  String get keyboardKeyNumpadEqual => 'Num =';

  @override
  String get keyboardKeyNumpadMultiply => 'Num *';

  @override
  String get keyboardKeyNumpadParenLeft => 'Num (';

  @override
  String get keyboardKeyNumpadParenRight => 'Num )';

  @override
  String get keyboardKeyNumpadSubtract => 'Num -';

  @override
  String get keyboardKeyPageDown => 'PgDown';

  @override
  String get keyboardKeyPageUp => 'PgUp';

  @override
  String get keyboardKeyPower => 'Power';

  @override
  String get keyboardKeyPowerOff => 'Power Off';

  @override
  String get keyboardKeyPrintScreen => 'Print Screen';

  @override
  String get keyboardKeyScrollLock => 'Scroll Lock';

  @override
  String get keyboardKeySelect => 'Select';

  @override
  String get keyboardKeySpace => 'Space';

  @override
  String get keyboardKeyShift => 'Shift';
}
