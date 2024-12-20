// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/services.dart';
/// @docImport 'package:flutter_localizations/flutter_localizations.dart';
///
/// @docImport 'about.dart';
/// @docImport 'action_buttons.dart';
/// @docImport 'app.dart';
/// @docImport 'app_bar.dart';
/// @docImport 'bottom_sheet.dart';
/// @docImport 'calendar_date_picker.dart';
/// @docImport 'chip.dart';
/// @docImport 'date_picker.dart';
/// @docImport 'expand_icon.dart';
/// @docImport 'expansion_tile.dart';
/// @docImport 'input_date_picker_form_field.dart';
/// @docImport 'paginated_data_table.dart';
/// @docImport 'popup_menu.dart';
/// @docImport 'refresh_indicator.dart';
/// @docImport 'reorderable_list.dart';
/// @docImport 'search_anchor.dart';
/// @docImport 'tabs.dart';
/// @docImport 'text_field.dart';
/// @docImport 'text_theme.dart';
/// @docImport 'theme_data.dart';
/// @docImport 'time_picker.dart';
/// @docImport 'user_accounts_drawer_header.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'debug.dart';
import 'time.dart';
import 'typography.dart';

// Examples can assume:
// late BuildContext context;

// ADDING A NEW STRING
//
// Please refer to instructions in this markdown file
// (packages/flutter_localizations/README.md)

/// Defines the localized resource values used by the Material widgets.
///
/// See also:
///
///  * [DefaultMaterialLocalizations], the default, English-only, implementation
///    of this interface.
///  * [GlobalMaterialLocalizations], which provides material localizations for
///    many languages.
abstract class MaterialLocalizations {
  /// The tooltip for the leading [AppBar] menu (a.k.a. 'hamburger') button.
  String get openAppDrawerTooltip;

  /// The [BackButton]'s tooltip.
  String get backButtonTooltip;

  /// The tooltip for the clear button to clear text on [SearchAnchor]'s search view.
  String get clearButtonTooltip;

  /// The [CloseButton]'s tooltip.
  String get closeButtonTooltip;

  /// The tooltip for the delete button on a [Chip].
  String get deleteButtonTooltip;

  /// The tooltip for the more button on an overflowing text selection menu.
  String get moreButtonTooltip;

  /// The tooltip for the [CalendarDatePicker]'s "next month" button.
  String get nextMonthTooltip;

  /// The tooltip for the [CalendarDatePicker]'s "previous month" button.
  String get previousMonthTooltip;

  /// The tooltip for the [PaginatedDataTable]'s "first page" button.
  String get firstPageTooltip;

  /// The tooltip for the [PaginatedDataTable]'s "last page" button.
  String get lastPageTooltip;

  /// The tooltip for the [PaginatedDataTable]'s "next page" button.
  String get nextPageTooltip;

  /// The tooltip for the [PaginatedDataTable]'s "previous page" button.
  String get previousPageTooltip;

  /// The default [PopupMenuButton] tooltip.
  String get showMenuTooltip;

  /// The default title for [AboutListTile].
  String aboutListTileTitle(String applicationName);

  /// Title for the [LicensePage] widget.
  String get licensesPageTitle;

  /// Subtitle for a package in the [LicensePage] widget.
  String licensesPackageDetailText(int licenseCount);

  /// Title for the [PaginatedDataTable]'s row info footer.
  String pageRowsInfoTitle(int firstRow, int lastRow, int rowCount, bool rowCountIsApproximate);

  /// Title for the [PaginatedDataTable]'s "rows per page" footer.
  String get rowsPerPageTitle;

  /// The accessibility label used on a tab in a [TabBar].
  ///
  /// This message describes the index of the selected tab and how many tabs
  /// there are, e.g. 'Tab 1 of 2' in United States English.
  ///
  /// `tabIndex` and `tabCount` must be greater than or equal to one.
  String tabLabel({required int tabIndex, required int tabCount});

  /// Title for the [PaginatedDataTable]'s selected row count header.
  String selectedRowCountTitle(int selectedRowCount);

  /// Label for "cancel" buttons and menu items.
  String get cancelButtonLabel;

  /// Label for "close" buttons and menu items.
  String get closeButtonLabel;

  /// Label for "continue" buttons and menu items.
  String get continueButtonLabel;

  /// Label for "copy" edit buttons and menu items.
  String get copyButtonLabel;

  /// Label for "cut" edit buttons and menu items.
  String get cutButtonLabel;

  /// Label for "scan text" OCR edit buttons and menu items.
  String get scanTextButtonLabel;

  /// Label for OK buttons and menu items.
  String get okButtonLabel;

  /// Label for "paste" edit buttons and menu items.
  String get pasteButtonLabel;

  /// Label for "select all" edit buttons and menu items.
  String get selectAllButtonLabel;

  /// Label for "look up" edit buttons and menu items.
  String get lookUpButtonLabel;

  /// Label for "search web" edit buttons and menu items.
  String get searchWebButtonLabel;

  /// Label for "share" edit buttons and menu items.
  String get shareButtonLabel;

  /// Label for the [AboutDialog] button that shows the [LicensePage].
  String get viewLicensesButtonLabel;

  /// The abbreviation for ante meridiem (before noon) shown in the time picker.
  String get anteMeridiemAbbreviation;

  /// The abbreviation for post meridiem (after noon) shown in the time picker.
  String get postMeridiemAbbreviation;

  /// The text-to-speech announcement made when a time picker invoked using
  /// [showTimePicker] is set to the hour picker mode.
  String get timePickerHourModeAnnouncement;

  /// The text-to-speech announcement made when a time picker invoked using
  /// [showTimePicker] is set to the minute picker mode.
  String get timePickerMinuteModeAnnouncement;

  /// Label read out by accessibility tools (TalkBack or VoiceOver) for a modal
  /// barrier to indicate that a tap dismisses the barrier.
  ///
  /// A modal barrier can for example be found behind an alert or popup to block
  /// user interaction with elements behind it.
  String get modalBarrierDismissLabel;

  /// Label read out by accessibility tools (TalkBack or VoiceOver) for a
  /// context menu to indicate that a tap dismisses the context menu.
  String get menuDismissLabel;

  /// Label read out by accessibility tools (TalkBack or VoiceOver) when a
  /// drawer widget is opened.
  String get drawerLabel;

  /// Label read out by accessibility tools (TalkBack or VoiceOver) when a
  /// popup menu widget is opened.
  String get popupMenuLabel;

  /// Label read out by accessibility tools (TalkBack or VoiceOver) when a
  /// MenuBarMenu widget is opened.
  String get menuBarMenuLabel;

  /// Label read out by accessibility tools (TalkBack or VoiceOver) when a
  /// dialog widget is opened.
  String get dialogLabel;

  /// Label read out by accessibility tools (TalkBack or VoiceOver) when an
  /// alert dialog widget is opened.
  String get alertDialogLabel;

  /// Label indicating that a text field is a search field. This will be used
  /// as a hint text in the text field.
  String get searchFieldLabel;

  /// Label indicating that a given date is the current date.
  String get currentDateLabel;

  /// The semantics label to describe the selected date in the calendar picker
  /// invoked using [showDatePicker].
  String get selectedDateLabel;

  /// Label for the scrim rendered underneath a [BottomSheet].
  String get scrimLabel;

  /// Label for a [BottomSheet], used as the `modalRouteContentName` of the
  /// [scrimOnTapHint].
  String get bottomSheetLabel;

  /// Hint text announced when tapping on the scrim underneath the content of
  /// a modal route.
  String scrimOnTapHint(String modalRouteContentName);

  /// The format used to lay out the time picker.
  ///
  /// The documentation for [TimeOfDayFormat] enum values provides details on
  /// each supported layout.
  TimeOfDayFormat timeOfDayFormat({bool alwaysUse24HourFormat = false});

  /// Defines the localized [TextStyle] geometry for [ThemeData.textTheme].
  ///
  /// The [scriptCategory] defines the overall geometry of a [TextTheme] for
  /// the [Typography.geometryThemeFor] method in terms of the
  /// three language categories defined in https://material.io/go/design-typography.
  ///
  /// Generally speaking, font sizes for [ScriptCategory.tall] and
  /// [ScriptCategory.dense] scripts - for text styles that are smaller than the
  /// title style - are one unit larger than they are for
  /// [ScriptCategory.englishLike] scripts.
  ScriptCategory get scriptCategory;

  /// Formats [number] as a decimal, inserting locale-appropriate thousands
  /// separators as necessary.
  String formatDecimal(int number);

  /// Formats [TimeOfDay.hour] in the given time of day according to the value
  /// of [timeOfDayFormat].
  ///
  /// If [alwaysUse24HourFormat] is true, formats hour using [HourFormat.HH]
  /// rather than the default for the current locale.
  String formatHour(TimeOfDay timeOfDay, {bool alwaysUse24HourFormat = false});

  /// Formats [TimeOfDay.minute] in the given time of day according to the value
  /// of [timeOfDayFormat].
  String formatMinute(TimeOfDay timeOfDay);

  /// Formats [timeOfDay] according to the value of [timeOfDayFormat].
  ///
  /// If [alwaysUse24HourFormat] is true, formats hour using [HourFormat.HH]
  /// rather than the default for the current locale. This value is usually
  /// passed from [MediaQueryData.alwaysUse24HourFormat], which has platform-
  /// specific behavior.
  String formatTimeOfDay(TimeOfDay timeOfDay, {bool alwaysUse24HourFormat = false});

  /// Full unabbreviated year format, e.g. 2017 rather than 17.
  String formatYear(DateTime date);

  /// Formats the date in a compact format.
  ///
  /// Usually just the numeric values for the for day, month and year are used.
  ///
  /// Examples:
  ///
  /// - US English: 02/21/2019
  /// - Russian: 21.02.2019
  ///
  /// See also:
  ///   * [parseCompactDate], which will convert a compact date string to a [DateTime].
  String formatCompactDate(DateTime date);

  /// Formats the date using a short-width format.
  ///
  /// Includes the abbreviation of the month, the day and year.
  ///
  /// Examples:
  ///
  /// - US English: Feb 21, 2019
  /// - Russian: 21 февр. 2019 г.
  String formatShortDate(DateTime date);

  /// Formats the date using a medium-width format.
  ///
  /// Abbreviates month and days of week. This appears in the header of the date
  /// picker invoked using [showDatePicker].
  ///
  /// Examples:
  ///
  /// - US English: Wed, Sep 27
  /// - Russian: ср, сент. 27
  String formatMediumDate(DateTime date);

  /// Formats day of week, month, day of month and year in a long-width format.
  ///
  /// Does not abbreviate names. Appears in spoken announcements of the date
  /// picker invoked using [showDatePicker], when accessibility mode is on.
  ///
  /// Examples:
  ///
  /// - US English: Wednesday, September 27, 2017
  /// - Russian: Среда, Сентябрь 27, 2017
  String formatFullDate(DateTime date);

  /// Formats the month and the year of the given [date].
  ///
  /// The returned string does not contain the day of the month. This appears
  /// in the date picker invoked using [showDatePicker].
  String formatMonthYear(DateTime date);

  /// Formats the month and day of the given [date].
  ///
  /// Examples:
  ///
  /// - US English: Feb 21
  /// - Russian: 21 февр.
  String formatShortMonthDay(DateTime date);

  /// Converts the given compact date formatted string into a [DateTime].
  ///
  /// The format of the string must be a valid compact date format for the
  /// given locale. If the text doesn't represent a valid date, `null` will be
  /// returned.
  ///
  /// See also:
  ///   * [formatCompactDate], which will convert a [DateTime] into a string in the compact format.
  DateTime? parseCompactDate(String? inputString);

  /// List of week day names in narrow format, usually 1- or 2-letter
  /// abbreviations of full names.
  ///
  /// The list begins with the value corresponding to Sunday and ends with
  /// Saturday. Use [firstDayOfWeekIndex] to find the first day of week in this
  /// list.
  ///
  /// Examples:
  ///
  /// - US English: S, M, T, W, T, F, S
  /// - Russian: вс, пн, вт, ср, чт, пт, сб - notice that the list begins with
  ///   вс (Sunday) even though the first day of week for Russian is Monday.
  List<String> get narrowWeekdays;

  /// Index of the first day of week, where 0 points to Sunday, and 6 points to
  /// Saturday.
  ///
  /// This getter is compatible with [narrowWeekdays]. For example:
  ///
  /// ```dart
  ///  MaterialLocalizations localizations = MaterialLocalizations.of(context);
  /// // The name of the first day of week for the current locale.
  /// String firstDayOfWeek = localizations.narrowWeekdays[localizations.firstDayOfWeekIndex];
  /// ```
  int get firstDayOfWeekIndex;

  /// The character string used to separate the parts of a compact date format
  /// (i.e. mm/dd/yyyy has a separator of '/').
  String get dateSeparator;

  /// The help text used on an empty [InputDatePickerFormField] to indicate
  /// to the user the date format being asked for.
  String get dateHelpText;

  /// The semantic label used to announce when the user has entered the year
  /// selection mode of the [CalendarDatePicker] which is used in the data picker
  /// dialog created with [showDatePicker].
  String get selectYearSemanticsLabel;

  /// The label used to indicate a date that has not been entered or selected
  /// yet in the date picker.
  String get unspecifiedDate;

  /// The label used to indicate a date range that has not been entered or
  /// selected yet in the date range picker.
  String get unspecifiedDateRange;

  /// The label used to describe the text field used in an [InputDatePickerFormField].
  String get dateInputLabel;

  /// The label used for the starting date input field in the date range picker
  /// created with [showDateRangePicker].
  String get dateRangeStartLabel;

  /// The label used for the ending date input field in the date range picker
  /// created with [showDateRangePicker].
  String get dateRangeEndLabel;

  /// The semantics label used for the selected start date in the date range
  /// picker's day grid.
  String dateRangeStartDateSemanticLabel(String formattedDate);

  /// The semantics label used for the selected end date in the date range
  /// picker's day grid.
  String dateRangeEndDateSemanticLabel(String formattedDate);

  /// Error message displayed to the user when they have entered a text string
  /// in an [InputDatePickerFormField] that is not in a valid date format.
  String get invalidDateFormatLabel;

  /// Error message displayed to the user when they have entered an invalid
  /// date range in the input mode of the date range picker created with
  /// [showDateRangePicker].
  String get invalidDateRangeLabel;

  /// Error message displayed to the user when they have entered a date that
  /// is outside the valid range for the date picker.
  /// [showDateRangePicker].
  String get dateOutOfRangeLabel;

  /// Label for a 'SAVE' button. Currently used by the full screen mode of the
  /// date range picker.
  String get saveButtonLabel;

  /// Label used in the header of the date picker dialog created with
  /// [showDatePicker].
  String get datePickerHelpText;

  /// Label used in the header of the date range picker dialog created with
  /// [showDateRangePicker].
  String get dateRangePickerHelpText;

  /// Tooltip used for the calendar mode button of the date pickers.
  String get calendarModeButtonLabel;

  /// Tooltip used for the text input mode button of the date pickers.
  String get inputDateModeButtonLabel;

  /// Label used in the header of the time picker dialog created with
  /// [showTimePicker] when in [TimePickerEntryMode.dial].
  String get timePickerDialHelpText;

  /// Label used in the header of the time picker dialog created with
  /// [showTimePicker] when in [TimePickerEntryMode.input].
  String get timePickerInputHelpText;

  /// Label used below the hour text field of the time picker dialog created
  /// with [showTimePicker] when in [TimePickerEntryMode.input].
  String get timePickerHourLabel;

  /// Label used below the minute text field of the time picker dialog created
  /// with [showTimePicker] when in [TimePickerEntryMode.input].
  String get timePickerMinuteLabel;

  /// Error message for the time picker dialog created with [showTimePicker]
  /// when in [TimePickerEntryMode.input].
  String get invalidTimeLabel;

  /// Tooltip used to put the time picker into [TimePickerEntryMode.dial].
  String get dialModeButtonLabel;

  /// Tooltip used to put the time picker into [TimePickerEntryMode.input].
  String get inputTimeModeButtonLabel;

  /// The semantics label used to indicate which account is signed in the
  /// [UserAccountsDrawerHeader] widget.
  String get signedInLabel;

  /// The semantics label used for the button on [UserAccountsDrawerHeader] that
  /// hides the list of accounts.
  String get hideAccountsLabel;

  /// The semantics label used for the button on [UserAccountsDrawerHeader] that
  /// shows the list of accounts.
  String get showAccountsLabel;

  /// The semantics label used for [ReorderableListView] to reorder an item in the
  /// list to the start of the list.
  @Deprecated(
    'Use the reorderItemToStart from WidgetsLocalizations instead. '
    'This feature was deprecated after v3.10.0-2.0.pre.',
  )
  String get reorderItemToStart;

  /// The semantics label used for [ReorderableListView] to reorder an item in the
  /// list to the end of the list.
  @Deprecated(
    'Use the reorderItemToEnd from WidgetsLocalizations instead. '
    'This feature was deprecated after v3.10.0-2.0.pre.',
  )
  String get reorderItemToEnd;

  /// The semantics label used for [ReorderableListView] to reorder an item in the
  /// list one space up the list.
  @Deprecated(
    'Use the reorderItemUp from WidgetsLocalizations instead. '
    'This feature was deprecated after v3.10.0-2.0.pre.',
  )
  String get reorderItemUp;

  /// The semantics label used for [ReorderableListView] to reorder an item in the
  /// list one space down the list.
  @Deprecated(
    'Use the reorderItemDown from WidgetsLocalizations instead. '
    'This feature was deprecated after v3.10.0-2.0.pre.',
  )
  String get reorderItemDown;

  /// The semantics label used for [ReorderableListView] to reorder an item in the
  /// list one space left in the list.
  @Deprecated(
    'Use the reorderItemLeft from WidgetsLocalizations instead. '
    'This feature was deprecated after v3.10.0-2.0.pre.',
  )
  String get reorderItemLeft;

  /// The semantics label used for [ReorderableListView] to reorder an item in the
  /// list one space right in the list.
  @Deprecated(
    'Use the reorderItemRight from WidgetsLocalizations instead. '
    'This feature was deprecated after v3.10.0-2.0.pre.',
  )
  String get reorderItemRight;

  /// The semantics hint to describe the tap action on an expanded [ExpandIcon].
  String get expandedIconTapHint => 'Collapse';

  /// The semantics hint to describe the tap action on a collapsed [ExpandIcon].
  String get collapsedIconTapHint => 'Expand';

  /// The semantics hint to describe the tap action on an expanded
  /// [ExpansionTile] on iOS and macOS. This is appended to the [collapsedHint]
  /// hint to provide a more detailed description of the action, e.g. "Expanded
  /// double tap to collapse".
  String get expansionTileExpandedHint => 'double tap to collapse';

  /// The semantics hint to describe the tap action on a collapsed
  /// [ExpansionTile] on iOS and macOS. This is appended to the [expandedHint]
  /// hint to provide a more detailed description of the action, e.g. "Collapsed
  /// double tap to expand".
  String get expansionTileCollapsedHint => 'double tap to expand';

  /// The semantics hint to describe the tap action on an expanded [ExpansionTile].
  String get expansionTileExpandedTapHint => 'Collapse';

  /// The semantics hint to describe the tap action on a collapsed [ExpansionTile].
  String get expansionTileCollapsedTapHint => 'Expand for more details';

  /// The semantics hint to describe the [ExpansionTile] expanded state.
  String get expandedHint => 'Collapsed';

  /// The semantics hint to describe the [ExpansionTile] collapsed state.
  String get collapsedHint => 'Expanded';

  /// The label for the [TextField]'s character counter.
  String remainingTextFieldCharacterCount(int remaining);

  /// The default semantics label for a [RefreshIndicator].
  String get refreshIndicatorSemanticLabel;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.alt].
  String get keyboardKeyAlt;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.altGraph].
  String get keyboardKeyAltGraph;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.backspace].
  String get keyboardKeyBackspace;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.capsLock].
  String get keyboardKeyCapsLock;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.channelDown].
  String get keyboardKeyChannelDown;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.channelUp].
  String get keyboardKeyChannelUp;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.control].
  String get keyboardKeyControl;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.delete].
  String get keyboardKeyDelete;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.eject].
  String get keyboardKeyEject;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.end].
  String get keyboardKeyEnd;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.escape].
  String get keyboardKeyEscape;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.fn].
  String get keyboardKeyFn;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.home].
  String get keyboardKeyHome;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.insert].
  String get keyboardKeyInsert;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.meta].
  String get keyboardKeyMeta;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.meta] on macOS.
  String get keyboardKeyMetaMacOs;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.meta] on Windows.
  String get keyboardKeyMetaWindows;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numLock].
  String get keyboardKeyNumLock;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad1].
  String get keyboardKeyNumpad1;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad2].
  String get keyboardKeyNumpad2;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad3].
  String get keyboardKeyNumpad3;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad4].
  String get keyboardKeyNumpad4;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad5].
  String get keyboardKeyNumpad5;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad6].
  String get keyboardKeyNumpad6;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad7].
  String get keyboardKeyNumpad7;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad8].
  String get keyboardKeyNumpad8;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad9].
  String get keyboardKeyNumpad9;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpad0].
  String get keyboardKeyNumpad0;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadAdd].
  String get keyboardKeyNumpadAdd;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadComma].
  String get keyboardKeyNumpadComma;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadDecimal].
  String get keyboardKeyNumpadDecimal;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadDivide].
  String get keyboardKeyNumpadDivide;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadEnter].
  String get keyboardKeyNumpadEnter;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadEqual].
  String get keyboardKeyNumpadEqual;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadMultiply].
  String get keyboardKeyNumpadMultiply;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadParenLeft].
  String get keyboardKeyNumpadParenLeft;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadParenRight].
  String get keyboardKeyNumpadParenRight;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.numpadSubtract].
  String get keyboardKeyNumpadSubtract;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.pageDown].
  String get keyboardKeyPageDown;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.pageUp].
  String get keyboardKeyPageUp;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.power].
  String get keyboardKeyPower;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.powerOff].
  String get keyboardKeyPowerOff;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.printScreen].
  String get keyboardKeyPrintScreen;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.scrollLock].
  String get keyboardKeyScrollLock;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.select].
  String get keyboardKeySelect;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.shift].
  String get keyboardKeyShift;

  /// The shortcut label for the keyboard key [LogicalKeyboardKey.space].
  String get keyboardKeySpace;

  /// The `MaterialLocalizations` from the closest [Localizations] instance
  /// that encloses the given context.
  ///
  /// If no [MaterialLocalizations] are available in the given `context`, this
  /// method throws an exception.
  ///
  /// This method is just a convenient shorthand for:
  /// `Localizations.of<MaterialLocalizations>(context, MaterialLocalizations)!`.
  ///
  /// References to the localized resources defined by this class are typically
  /// written in terms of this method. For example:
  ///
  /// ```dart
  /// tooltip: MaterialLocalizations.of(context).backButtonTooltip,
  /// ```
  static MaterialLocalizations of(BuildContext context) {
    assert(debugCheckHasMaterialLocalizations(context));
    return Localizations.of<MaterialLocalizations>(context, MaterialLocalizations)!;
  }
}

class _MaterialLocalizationsDelegate extends LocalizationsDelegate<MaterialLocalizations> {
  const _MaterialLocalizationsDelegate();

  @override
  bool isSupported(Locale locale) => locale.languageCode == 'en';

  @override
  Future<MaterialLocalizations> load(Locale locale) => DefaultMaterialLocalizations.load(locale);

  @override
  bool shouldReload(_MaterialLocalizationsDelegate old) => false;

  @override
  String toString() => 'DefaultMaterialLocalizations.delegate(en_US)';
}

/// US English strings for the material widgets.
///
/// See also:
///
///  * [GlobalMaterialLocalizations], which provides material localizations for
///    many languages.
///  * [MaterialApp.localizationsDelegates], which automatically includes
///    [DefaultMaterialLocalizations.delegate] by default.
class DefaultMaterialLocalizations implements MaterialLocalizations {
  /// Constructs an object that defines the material widgets' localized strings
  /// for US English (only).
  ///
  /// [LocalizationsDelegate] implementations typically call the static [load]
  /// function, rather than constructing this class directly.
  const DefaultMaterialLocalizations();

  // Ordered to match DateTime.monday=1, DateTime.sunday=6
  static const List<String> _shortWeekdays = <String>[
    'Mon',
    'Tue',
    'Wed',
    'Thu',
    'Fri',
    'Sat',
    'Sun',
  ];

  // Ordered to match DateTime.monday=1, DateTime.sunday=6
  static const List<String> _weekdays = <String>[
    'Monday',
    'Tuesday',
    'Wednesday',
    'Thursday',
    'Friday',
    'Saturday',
    'Sunday',
  ];

  static const List<String> _narrowWeekdays = <String>['S', 'M', 'T', 'W', 'T', 'F', 'S'];

  static const List<String> _shortMonths = <String>[
    'Jan',
    'Feb',
    'Mar',
    'Apr',
    'May',
    'Jun',
    'Jul',
    'Aug',
    'Sep',
    'Oct',
    'Nov',
    'Dec',
  ];

  static const List<String> _months = <String>[
    'January',
    'February',
    'March',
    'April',
    'May',
    'June',
    'July',
    'August',
    'September',
    'October',
    'November',
    'December',
  ];

  /// Returns the number of days in a month, according to the proleptic
  /// Gregorian calendar.
  ///
  /// This applies the leap year logic introduced by the Gregorian reforms of
  /// 1582. It will not give valid results for dates prior to that time.
  int _getDaysInMonth(int year, int month) {
    if (month == DateTime.february) {
      final bool isLeapYear = (year % 4 == 0) && (year % 100 != 0) || (year % 400 == 0);
      if (isLeapYear) {
        return 29;
      }
      return 28;
    }
    const List<int> daysInMonth = <int>[31, -1, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    return daysInMonth[month - 1];
  }

  @override
  String formatHour(TimeOfDay timeOfDay, {bool alwaysUse24HourFormat = false}) {
    final TimeOfDayFormat format = timeOfDayFormat(alwaysUse24HourFormat: alwaysUse24HourFormat);
    switch (format) {
      case TimeOfDayFormat.h_colon_mm_space_a:
        return formatDecimal(timeOfDay.hourOfPeriod == 0 ? 12 : timeOfDay.hourOfPeriod);
      case TimeOfDayFormat.HH_colon_mm:
        return _formatTwoDigitZeroPad(timeOfDay.hour);
      case TimeOfDayFormat.a_space_h_colon_mm:
      case TimeOfDayFormat.frenchCanadian:
      case TimeOfDayFormat.H_colon_mm:
      case TimeOfDayFormat.HH_dot_mm:
        throw AssertionError('$runtimeType does not support $format.');
    }
  }

  /// Formats [number] using two digits, assuming it's in the 0-99 inclusive
  /// range. Not designed to format values outside this range.
  String _formatTwoDigitZeroPad(int number) {
    assert(0 <= number && number < 100);

    if (number < 10) {
      return '0$number';
    }

    return '$number';
  }

  @override
  String formatMinute(TimeOfDay timeOfDay) {
    final int minute = timeOfDay.minute;
    return minute < 10 ? '0$minute' : minute.toString();
  }

  @override
  String formatYear(DateTime date) => date.year.toString();

  @override
  String formatCompactDate(DateTime date) {
    // Assumes US mm/dd/yyyy format
    final String month = _formatTwoDigitZeroPad(date.month);
    final String day = _formatTwoDigitZeroPad(date.day);
    final String year = date.year.toString().padLeft(4, '0');
    return '$month/$day/$year';
  }

  @override
  String formatShortDate(DateTime date) {
    final String month = _shortMonths[date.month - DateTime.january];
    return '$month ${date.day}, ${date.year}';
  }

  @override
  String formatMediumDate(DateTime date) {
    final String day = _shortWeekdays[date.weekday - DateTime.monday];
    final String month = _shortMonths[date.month - DateTime.january];
    return '$day, $month ${date.day}';
  }

  @override
  String formatFullDate(DateTime date) {
    final String month = _months[date.month - DateTime.january];
    return '${_weekdays[date.weekday - DateTime.monday]}, $month ${date.day}, ${date.year}';
  }

  @override
  String formatMonthYear(DateTime date) {
    final String year = formatYear(date);
    final String month = _months[date.month - DateTime.january];
    return '$month $year';
  }

  @override
  String formatShortMonthDay(DateTime date) {
    final String month = _shortMonths[date.month - DateTime.january];
    return '$month ${date.day}';
  }

  @override
  DateTime? parseCompactDate(String? inputString) {
    if (inputString == null) {
      return null;
    }

    // Assumes US mm/dd/yyyy format
    final List<String> inputParts = inputString.split('/');
    if (inputParts.length != 3) {
      return null;
    }

    final int? year = int.tryParse(inputParts[2], radix: 10);
    if (year == null || year < 1) {
      return null;
    }

    final int? month = int.tryParse(inputParts[0], radix: 10);
    if (month == null || month < 1 || month > 12) {
      return null;
    }

    final int? day = int.tryParse(inputParts[1], radix: 10);
    if (day == null || day < 1 || day > _getDaysInMonth(year, month)) {
      return null;
    }

    try {
      return DateTime(year, month, day);
    } on ArgumentError {
      return null;
    }
  }

  @override
  List<String> get narrowWeekdays => _narrowWeekdays;

  @override
  int get firstDayOfWeekIndex => 0; // narrowWeekdays[0] is 'S' for Sunday

  @override
  String get dateSeparator => '/';

  @override
  String get dateHelpText => 'mm/dd/yyyy';

  @override
  String get selectYearSemanticsLabel => 'Select year';

  @override
  String get unspecifiedDate => 'Date';

  @override
  String get unspecifiedDateRange => 'Date Range';

  @override
  String get dateInputLabel => 'Enter Date';

  @override
  String get dateRangeStartLabel => 'Start Date';

  @override
  String get dateRangeEndLabel => 'End Date';

  @override
  String dateRangeStartDateSemanticLabel(String formattedDate) => 'Start date $formattedDate';

  @override
  String dateRangeEndDateSemanticLabel(String formattedDate) => 'End date $formattedDate';

  @override
  String get invalidDateFormatLabel => 'Invalid format.';

  @override
  String get invalidDateRangeLabel => 'Invalid range.';

  @override
  String get dateOutOfRangeLabel => 'Out of range.';

  @override
  String get saveButtonLabel => 'Save';

  @override
  String get datePickerHelpText => 'Select date';

  @override
  String get dateRangePickerHelpText => 'Select range';

  @override
  String get calendarModeButtonLabel => 'Switch to calendar';

  @override
  String get inputDateModeButtonLabel => 'Switch to input';

  @override
  String get timePickerDialHelpText => 'Select time';

  @override
  String get timePickerInputHelpText => 'Enter time';

  @override
  String get timePickerHourLabel => 'Hour';

  @override
  String get timePickerMinuteLabel => 'Minute';

  @override
  String get invalidTimeLabel => 'Enter a valid time';

  @override
  String get dialModeButtonLabel => 'Switch to dial picker mode';

  @override
  String get inputTimeModeButtonLabel => 'Switch to text input mode';

  String _formatDayPeriod(TimeOfDay timeOfDay) {
    return switch (timeOfDay.period) {
      DayPeriod.am => anteMeridiemAbbreviation,
      DayPeriod.pm => postMeridiemAbbreviation,
    };
  }

  @override
  String formatDecimal(int number) {
    if (number > -1000 && number < 1000) {
      return number.toString();
    }

    final String digits = number.abs().toString();
    final StringBuffer result = StringBuffer(number < 0 ? '-' : '');
    final int maxDigitIndex = digits.length - 1;
    for (int i = 0; i <= maxDigitIndex; i += 1) {
      result.write(digits[i]);
      if (i < maxDigitIndex && (maxDigitIndex - i) % 3 == 0) {
        result.write(',');
      }
    }
    return result.toString();
  }

  @override
  String formatTimeOfDay(TimeOfDay timeOfDay, {bool alwaysUse24HourFormat = false}) {
    // Not using intl.DateFormat for two reasons:
    //
    // - DateFormat supports more formats than our material time picker does,
    //   and we want to be consistent across time picker format and the string
    //   formatting of the time of day.
    // - DateFormat operates on DateTime, which is sensitive to time eras and
    //   time zones, while here we want to format hour and minute within one day
    //   no matter what date the day falls on.
    final StringBuffer buffer = StringBuffer();

    // Add hour:minute.
    buffer
      ..write(formatHour(timeOfDay, alwaysUse24HourFormat: alwaysUse24HourFormat))
      ..write(':')
      ..write(formatMinute(timeOfDay));

    if (alwaysUse24HourFormat) {
      // There's no AM/PM indicator in 24-hour format.
      return '$buffer';
    }

    // Add AM/PM indicator.
    buffer
      ..write(' ')
      ..write(_formatDayPeriod(timeOfDay));
    return '$buffer';
  }

  @override
  String get openAppDrawerTooltip => 'Open navigation menu';

  @override
  String get backButtonTooltip => 'Back';

  @override
  String get clearButtonTooltip => 'Clear text';

  @override
  String get closeButtonTooltip => 'Close';

  @override
  String get deleteButtonTooltip => 'Delete';

  @override
  String get moreButtonTooltip => 'More';

  @override
  String get nextMonthTooltip => 'Next month';

  @override
  String get previousMonthTooltip => 'Previous month';

  @override
  String get nextPageTooltip => 'Next page';

  @override
  String get previousPageTooltip => 'Previous page';

  @override
  String get firstPageTooltip => 'First page';

  @override
  String get lastPageTooltip => 'Last page';

  @override
  String get showMenuTooltip => 'Show menu';

  @override
  String get drawerLabel => 'Navigation menu';

  @override
  String get menuBarMenuLabel => 'Menu bar menu';

  @override
  String get popupMenuLabel => 'Popup menu';

  @override
  String get dialogLabel => 'Dialog';

  @override
  String get alertDialogLabel => 'Alert';

  @override
  String get searchFieldLabel => 'Search';

  @override
  String get currentDateLabel => 'Today';

  @override
  String get selectedDateLabel => 'Selected';

  @override
  String get scrimLabel => 'Scrim';

  @override
  String get bottomSheetLabel => 'Bottom Sheet';

  @override
  String scrimOnTapHint(String modalRouteContentName) => 'Close $modalRouteContentName';

  @override
  String aboutListTileTitle(String applicationName) => 'About $applicationName';

  @override
  String get licensesPageTitle => 'Licenses';

  @override
  String licensesPackageDetailText(int licenseCount) {
    assert(licenseCount >= 0);
    return switch (licenseCount) {
      0 => 'No licenses.',
      1 => '1 license.',
      _ => '$licenseCount licenses.',
    };
  }

  @override
  String pageRowsInfoTitle(int firstRow, int lastRow, int rowCount, bool rowCountIsApproximate) {
    return rowCountIsApproximate
        ? '$firstRow–$lastRow of about $rowCount'
        : '$firstRow–$lastRow of $rowCount';
  }

  @override
  String get rowsPerPageTitle => 'Rows per page:';

  @override
  String tabLabel({required int tabIndex, required int tabCount}) {
    assert(tabIndex >= 1);
    assert(tabCount >= 1);
    return 'Tab $tabIndex of $tabCount';
  }

  @override
  String selectedRowCountTitle(int selectedRowCount) {
    return switch (selectedRowCount) {
      0 => 'No items selected',
      1 => '1 item selected',
      _ => '$selectedRowCount items selected',
    };
  }

  @override
  String get cancelButtonLabel => 'Cancel';

  @override
  String get closeButtonLabel => 'Close';

  @override
  String get continueButtonLabel => 'Continue';

  @override
  String get copyButtonLabel => 'Copy';

  @override
  String get cutButtonLabel => 'Cut';

  @override
  String get scanTextButtonLabel => 'Scan text';

  @override
  String get okButtonLabel => 'OK';

  @override
  String get pasteButtonLabel => 'Paste';

  @override
  String get selectAllButtonLabel => 'Select all';

  @override
  String get lookUpButtonLabel => 'Look Up';

  @override
  String get searchWebButtonLabel => 'Search Web';

  @override
  String get shareButtonLabel => 'Share';

  @override
  String get viewLicensesButtonLabel => 'View licenses';

  @override
  String get anteMeridiemAbbreviation => 'AM';

  @override
  String get postMeridiemAbbreviation => 'PM';

  @override
  String get timePickerHourModeAnnouncement => 'Select hours';

  @override
  String get timePickerMinuteModeAnnouncement => 'Select minutes';

  @override
  String get modalBarrierDismissLabel => 'Dismiss';

  @override
  String get menuDismissLabel => 'Dismiss menu';

  @override
  ScriptCategory get scriptCategory => ScriptCategory.englishLike;

  @override
  TimeOfDayFormat timeOfDayFormat({bool alwaysUse24HourFormat = false}) {
    return alwaysUse24HourFormat ? TimeOfDayFormat.HH_colon_mm : TimeOfDayFormat.h_colon_mm_space_a;
  }

  @override
  String get signedInLabel => 'Signed in';

  @override
  String get hideAccountsLabel => 'Hide accounts';

  @override
  String get showAccountsLabel => 'Show accounts';

  @override
  String get reorderItemUp => 'Move up';

  @override
  String get reorderItemDown => 'Move down';

  @override
  String get reorderItemLeft => 'Move left';

  @override
  String get reorderItemRight => 'Move right';

  @override
  String get reorderItemToEnd => 'Move to the end';

  @override
  String get reorderItemToStart => 'Move to the start';

  @override
  String get expandedIconTapHint => 'Collapse';

  @override
  String get collapsedIconTapHint => 'Expand';

  @override
  String get expansionTileExpandedHint => 'double tap to collapse';

  @override
  String get expansionTileCollapsedHint => 'double tap to expand';

  @override
  String get expansionTileExpandedTapHint => 'Collapse';

  @override
  String get expansionTileCollapsedTapHint => 'Expand for more details';

  @override
  String get expandedHint => 'Collapsed';

  @override
  String get collapsedHint => 'Expanded';

  @override
  String get refreshIndicatorSemanticLabel => 'Refresh';

  /// Creates an object that provides US English resource values for the material
  /// library widgets.
  ///
  /// The [locale] parameter is ignored.
  ///
  /// This method is typically used to create a [LocalizationsDelegate].
  /// The [MaterialApp] does so by default.
  static Future<MaterialLocalizations> load(Locale locale) {
    return SynchronousFuture<MaterialLocalizations>(const DefaultMaterialLocalizations());
  }

  /// A [LocalizationsDelegate] that uses [DefaultMaterialLocalizations.load]
  /// to create an instance of this class.
  ///
  /// [MaterialApp] automatically adds this value to [MaterialApp.localizationsDelegates].
  static const LocalizationsDelegate<MaterialLocalizations> delegate =
      _MaterialLocalizationsDelegate();

  @override
  String remainingTextFieldCharacterCount(int remaining) {
    return switch (remaining) {
      0 => 'No characters remaining',
      1 => '1 character remaining',
      _ => '$remaining characters remaining',
    };
  }

  @override
  String get keyboardKeyAlt => 'Alt';

  @override
  String get keyboardKeyAltGraph => 'AltGr';

  @override
  String get keyboardKeyBackspace => 'Backspace';

  @override
  String get keyboardKeyCapsLock => 'Caps Lock';

  @override
  String get keyboardKeyChannelDown => 'Channel Down';

  @override
  String get keyboardKeyChannelUp => 'Channel Up';

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
  String get keyboardKeyNumpad0 => 'Num 0';

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
  String get keyboardKeyShift => 'Shift';

  @override
  String get keyboardKeySpace => 'Space';
}
