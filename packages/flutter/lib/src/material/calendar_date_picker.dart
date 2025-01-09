// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'date_picker.dart';
/// @docImport 'time_picker.dart';
library;

import 'dart:math' as math;

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'color_scheme.dart';
import 'date.dart';
import 'date_picker_theme.dart';
import 'debug.dart';
import 'divider.dart';
import 'icon_button.dart';
import 'icons.dart';
import 'ink_decoration.dart';
import 'ink_well.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'text_theme.dart';
import 'theme.dart';

const Duration _monthScrollDuration = Duration(milliseconds: 200);

const double _dayPickerRowHeight = 42.0;
const int _maxDayPickerRowCount = 6; // A 31 day month that starts on Saturday.
// One extra row for the day-of-week header.
const double _maxDayPickerHeight = _dayPickerRowHeight * (_maxDayPickerRowCount + 1);
const double _monthPickerHorizontalPadding = 8.0;

const int _yearPickerColumnCount = 3;
const double _yearPickerPadding = 16.0;
const double _yearPickerRowHeight = 52.0;
const double _yearPickerRowSpacing = 8.0;

const double _subHeaderHeight = 52.0;
const double _monthNavButtonsWidth = 108.0;

// 3.0 is the maximum scale factor on mobile phones. As of 07/30/24, iOS goes up
// to a max of 3.0 text scale factor, and Android goes up to 2.0. This is the
// default used for non-range date pickers. This default is changed to a lower
// value at different parts of the date pickers depending on content, and device
// orientation.
const double _kMaxTextScaleFactor = 3.0;

const double _kModeToggleButtonMaxScaleFactor = 2.0;

// The max scale factor of the day picker grid. This affects the size of the
// individual days in calendar view. Due to them filling a majority of the modal,
// which covers most of the screen, there's a limit in how large they can grow.
// There is also less room vertically in landscape orientation.
const double _kDayPickerGridPortraitMaxScaleFactor = 2.0;
const double _kDayPickerGridLandscapeMaxScaleFactor = 1.5;

// 14 is a common font size used to compute the effective text scale.
const double _fontSizeToScale = 14.0;

/// Displays a grid of days for a given month and allows the user to select a
/// date.
///
/// Days are arranged in a rectangular grid with one column for each day of the
/// week. Controls are provided to change the year and month that the grid is
/// showing.
///
/// The calendar picker widget is rarely used directly. Instead, consider using
/// [showDatePicker], which will create a dialog that uses this as well as
/// provides a text entry option.
///
/// See also:
///
///  * [showDatePicker], which creates a Dialog that contains a
///    [CalendarDatePicker] and provides an optional compact view where the
///    user can enter a date as a line of text.
///  * [showTimePicker], which shows a dialog that contains a Material Design
///    time picker.
///
class CalendarDatePicker extends StatefulWidget {
  /// Creates a calendar date picker.
  ///
  /// It will display a grid of days for the [initialDate]'s month, or, if that
  /// is null, the [currentDate]'s month. The day indicated by [initialDate] will
  /// be selected if it is not null.
  ///
  /// The optional [onDisplayedMonthChanged] callback can be used to track
  /// the currently displayed month.
  ///
  /// The user interface provides a way to change the year of the month being
  /// displayed. By default it will show the day grid, but this can be changed
  /// to start in the year selection interface with [initialCalendarMode] set
  /// to [DatePickerMode.year].
  ///
  /// The [lastDate] must be after or equal to [firstDate].
  ///
  /// The [initialDate], if provided, must be between [firstDate] and [lastDate]
  /// or equal to one of them.
  ///
  /// The [currentDate] represents the current day (i.e. today). This
  /// date will be highlighted in the day grid. If null, the date of
  /// `DateTime.now()` will be used.
  ///
  /// If [selectableDayPredicate] and [initialDate] are both non-null,
  /// [selectableDayPredicate] must return `true` for the [initialDate].
  CalendarDatePicker({
    super.key,
    required DateTime? initialDate,
    required DateTime firstDate,
    required DateTime lastDate,
    DateTime? currentDate,
    required this.onDateChanged,
    this.onDisplayedMonthChanged,
    this.initialCalendarMode = DatePickerMode.day,
    this.selectableDayPredicate,
  }) : initialDate = initialDate == null ? null : DateUtils.dateOnly(initialDate),
       firstDate = DateUtils.dateOnly(firstDate),
       lastDate = DateUtils.dateOnly(lastDate),
       currentDate = DateUtils.dateOnly(currentDate ?? DateTime.now()) {
    assert(
      !this.lastDate.isBefore(this.firstDate),
      'lastDate ${this.lastDate} must be on or after firstDate ${this.firstDate}.',
    );
    assert(
      this.initialDate == null || !this.initialDate!.isBefore(this.firstDate),
      'initialDate ${this.initialDate} must be on or after firstDate ${this.firstDate}.',
    );
    assert(
      this.initialDate == null || !this.initialDate!.isAfter(this.lastDate),
      'initialDate ${this.initialDate} must be on or before lastDate ${this.lastDate}.',
    );
    assert(
      selectableDayPredicate == null ||
          this.initialDate == null ||
          selectableDayPredicate!(this.initialDate!),
      'Provided initialDate ${this.initialDate} must satisfy provided selectableDayPredicate.',
    );
  }

  /// The initially selected [DateTime] that the picker should display.
  ///
  /// Subsequently changing this has no effect. To change the selected date,
  /// change the [key] to create a new instance of the [CalendarDatePicker], and
  /// provide that widget the new [initialDate]. This will reset the widget's
  /// interactive state.
  final DateTime? initialDate;

  /// The earliest allowable [DateTime] that the user can select.
  final DateTime firstDate;

  /// The latest allowable [DateTime] that the user can select.
  final DateTime lastDate;

  /// The [DateTime] representing today. It will be highlighted in the day grid.
  final DateTime currentDate;

  /// Called when the user selects a date in the picker.
  final ValueChanged<DateTime> onDateChanged;

  /// Called when the user navigates to a new month/year in the picker.
  final ValueChanged<DateTime>? onDisplayedMonthChanged;

  /// The initial display of the calendar picker.
  ///
  /// Subsequently changing this has no effect. To change the calendar mode,
  /// change the [key] to create a new instance of the [CalendarDatePicker], and
  /// provide that widget a new [initialCalendarMode]. This will reset the
  /// widget's interactive state.
  final DatePickerMode initialCalendarMode;

  /// Function to provide full control over which dates in the calendar can be selected.
  final SelectableDayPredicate? selectableDayPredicate;

  @override
  State<CalendarDatePicker> createState() => _CalendarDatePickerState();
}

class _CalendarDatePickerState extends State<CalendarDatePicker> {
  bool _announcedInitialDate = false;
  late DatePickerMode _mode;
  late DateTime _currentDisplayedMonthDate;
  DateTime? _selectedDate;
  final GlobalKey _monthPickerKey = GlobalKey();
  final GlobalKey _yearPickerKey = GlobalKey();
  late MaterialLocalizations _localizations;
  late TextDirection _textDirection;

  @override
  void initState() {
    super.initState();
    _mode = widget.initialCalendarMode;
    final DateTime currentDisplayedDate = widget.initialDate ?? widget.currentDate;
    _currentDisplayedMonthDate = DateTime(currentDisplayedDate.year, currentDisplayedDate.month);
    if (widget.initialDate != null) {
      _selectedDate = widget.initialDate;
    }
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    assert(debugCheckHasMaterial(context));
    assert(debugCheckHasMaterialLocalizations(context));
    assert(debugCheckHasDirectionality(context));
    _localizations = MaterialLocalizations.of(context);
    _textDirection = Directionality.of(context);
    if (!_announcedInitialDate && widget.initialDate != null) {
      assert(_selectedDate != null);
      _announcedInitialDate = true;
      final bool isToday = DateUtils.isSameDay(widget.currentDate, _selectedDate);
      final String semanticLabelSuffix = isToday ? ', ${_localizations.currentDateLabel}' : '';
      SemanticsService.announce(
        '${_localizations.formatFullDate(_selectedDate!)}$semanticLabelSuffix',
        _textDirection,
      );
    }
  }

  void _vibrate() {
    switch (Theme.of(context).platform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        HapticFeedback.vibrate();
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        break;
    }
  }

  void _handleModeChanged(DatePickerMode mode) {
    _vibrate();
    setState(() {
      _mode = mode;
      if (_selectedDate case final DateTime selected) {
        final String message = switch (mode) {
          DatePickerMode.day => _localizations.formatMonthYear(selected),
          DatePickerMode.year => _localizations.formatYear(selected),
        };
        SemanticsService.announce(message, _textDirection);
      }
    });
  }

  void _handleMonthChanged(DateTime date) {
    setState(() {
      if (_currentDisplayedMonthDate.year != date.year ||
          _currentDisplayedMonthDate.month != date.month) {
        _currentDisplayedMonthDate = DateTime(date.year, date.month);
        widget.onDisplayedMonthChanged?.call(_currentDisplayedMonthDate);
      }
    });
  }

  void _handleYearChanged(DateTime value) {
    _vibrate();

    final int daysInMonth = DateUtils.getDaysInMonth(value.year, value.month);
    final int preferredDay = math.min(_selectedDate?.day ?? 1, daysInMonth);
    value = value.copyWith(day: preferredDay);

    if (value.isBefore(widget.firstDate)) {
      value = widget.firstDate;
    } else if (value.isAfter(widget.lastDate)) {
      value = widget.lastDate;
    }

    setState(() {
      _mode = DatePickerMode.day;
      _handleMonthChanged(value);

      if (_isSelectable(value)) {
        _selectedDate = value;
        widget.onDateChanged(_selectedDate!);
      }
    });
  }

  void _handleDayChanged(DateTime value) {
    _vibrate();
    setState(() {
      _selectedDate = value;
      widget.onDateChanged(_selectedDate!);
      switch (Theme.of(context).platform) {
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          final bool isToday = DateUtils.isSameDay(widget.currentDate, _selectedDate);
          final String semanticLabelSuffix = isToday ? ', ${_localizations.currentDateLabel}' : '';
          SemanticsService.announce(
            '${_localizations.selectedDateLabel} ${_localizations.formatFullDate(_selectedDate!)}$semanticLabelSuffix',
            _textDirection,
          );
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
          break;
      }
    });
  }

  bool _isSelectable(DateTime date) {
    return widget.selectableDayPredicate?.call(date) ?? true;
  }

  Widget _buildPicker() {
    switch (_mode) {
      case DatePickerMode.day:
        return _MonthPicker(
          key: _monthPickerKey,
          initialMonth: _currentDisplayedMonthDate,
          currentDate: widget.currentDate,
          firstDate: widget.firstDate,
          lastDate: widget.lastDate,
          selectedDate: _selectedDate,
          onChanged: _handleDayChanged,
          onDisplayedMonthChanged: _handleMonthChanged,
          selectableDayPredicate: widget.selectableDayPredicate,
        );
      case DatePickerMode.year:
        return Padding(
          padding: const EdgeInsets.only(top: _subHeaderHeight),
          child: YearPicker(
            key: _yearPickerKey,
            currentDate: widget.currentDate,
            firstDate: widget.firstDate,
            lastDate: widget.lastDate,
            selectedDate: _currentDisplayedMonthDate,
            onChanged: _handleYearChanged,
          ),
        );
    }
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    assert(debugCheckHasMaterialLocalizations(context));
    assert(debugCheckHasDirectionality(context));
    final double textScaleFactor =
        MediaQuery.textScalerOf(
          context,
        ).clamp(maxScaleFactor: _kMaxTextScaleFactor).scale(_fontSizeToScale) /
        _fontSizeToScale;
    // Scale the height of the picker area up with larger text. The size of the
    // picker has room for larger text, up until a scale facotr of 1.3. After
    // after which, we increase the height to add room for content to continue
    // to scale the text size.
    final double scaledMaxDayPickerHeight =
        textScaleFactor > 1.3
            ? _maxDayPickerHeight + ((_maxDayPickerRowCount + 1) * ((textScaleFactor - 1) * 8))
            : _maxDayPickerHeight;
    return Stack(
      children: <Widget>[
        SizedBox(height: _subHeaderHeight + scaledMaxDayPickerHeight, child: _buildPicker()),
        // Put the mode toggle button on top so that it won't be covered up by the _MonthPicker
        MediaQuery.withClampedTextScaling(
          maxScaleFactor: _kModeToggleButtonMaxScaleFactor,
          child: _DatePickerModeToggleButton(
            mode: _mode,
            title: _localizations.formatMonthYear(_currentDisplayedMonthDate),
            onTitlePressed:
                () => _handleModeChanged(switch (_mode) {
                  DatePickerMode.day => DatePickerMode.year,
                  DatePickerMode.year => DatePickerMode.day,
                }),
          ),
        ),
      ],
    );
  }
}

/// A button that used to toggle the [DatePickerMode] for a date picker.
///
/// This appears above the calendar grid and allows the user to toggle the
/// [DatePickerMode] to display either the calendar view or the year list.
class _DatePickerModeToggleButton extends StatefulWidget {
  const _DatePickerModeToggleButton({
    required this.mode,
    required this.title,
    required this.onTitlePressed,
  });

  /// The current display of the calendar picker.
  final DatePickerMode mode;

  /// The text that displays the current month/year being viewed.
  final String title;

  /// The callback when the title is pressed.
  final VoidCallback onTitlePressed;

  @override
  _DatePickerModeToggleButtonState createState() => _DatePickerModeToggleButtonState();
}

class _DatePickerModeToggleButtonState extends State<_DatePickerModeToggleButton>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      value: widget.mode == DatePickerMode.year ? 0.5 : 0,
      upperBound: 0.5,
      duration: const Duration(milliseconds: 200),
      vsync: this,
    );
  }

  @override
  void didUpdateWidget(_DatePickerModeToggleButton oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.mode == widget.mode) {
      return;
    }

    if (widget.mode == DatePickerMode.year) {
      _controller.forward();
    } else {
      _controller.reverse();
    }
  }

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final TextTheme textTheme = Theme.of(context).textTheme;
    final Color controlColor = colorScheme.onSurface.withOpacity(0.60);

    return SizedBox(
      height: _subHeaderHeight,
      child: Padding(
        padding: const EdgeInsetsDirectional.only(start: 16, end: 4),
        child: Row(
          children: <Widget>[
            Flexible(
              child: Semantics(
                label: MaterialLocalizations.of(context).selectYearSemanticsLabel,
                button: true,
                container: true,
                child: SizedBox(
                  height: _subHeaderHeight,
                  child: InkWell(
                    onTap: widget.onTitlePressed,
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 8),
                      child: Row(
                        children: <Widget>[
                          Flexible(
                            child: Text(
                              widget.title,
                              overflow: TextOverflow.ellipsis,
                              style: textTheme.titleSmall?.copyWith(color: controlColor),
                            ),
                          ),
                          RotationTransition(
                            turns: _controller,
                            child: Icon(Icons.arrow_drop_down, color: controlColor),
                          ),
                        ],
                      ),
                    ),
                  ),
                ),
              ),
            ),
            if (widget.mode == DatePickerMode.day)
              // Give space for the prev/next month buttons that are underneath this row
              const SizedBox(width: _monthNavButtonsWidth),
          ],
        ),
      ),
    );
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }
}

class _MonthPicker extends StatefulWidget {
  /// Creates a month picker.
  _MonthPicker({
    super.key,
    required this.initialMonth,
    required this.currentDate,
    required this.firstDate,
    required this.lastDate,
    required this.selectedDate,
    required this.onChanged,
    required this.onDisplayedMonthChanged,
    this.selectableDayPredicate,
  }) : assert(!firstDate.isAfter(lastDate)),
       assert(selectedDate == null || !selectedDate.isBefore(firstDate)),
       assert(selectedDate == null || !selectedDate.isAfter(lastDate));

  /// The initial month to display.
  ///
  /// Subsequently changing this has no effect. To change the selected month,
  /// change the [key] to create a new instance of the [_MonthPicker], and
  /// provide that widget the new [initialMonth]. This will reset the widget's
  /// interactive state.
  final DateTime initialMonth;

  /// The current date.
  ///
  /// This date is subtly highlighted in the picker.
  final DateTime currentDate;

  /// The earliest date the user is permitted to pick.
  ///
  /// This date must be on or before the [lastDate].
  final DateTime firstDate;

  /// The latest date the user is permitted to pick.
  ///
  /// This date must be on or after the [firstDate].
  final DateTime lastDate;

  /// The currently selected date.
  ///
  /// This date is highlighted in the picker.
  final DateTime? selectedDate;

  /// Called when the user picks a day.
  final ValueChanged<DateTime> onChanged;

  /// Called when the user navigates to a new month.
  final ValueChanged<DateTime> onDisplayedMonthChanged;

  /// Optional user supplied predicate function to customize selectable days.
  final SelectableDayPredicate? selectableDayPredicate;

  @override
  _MonthPickerState createState() => _MonthPickerState();
}

class _MonthPickerState extends State<_MonthPicker> {
  final GlobalKey _pageViewKey = GlobalKey();
  late DateTime _currentMonth;
  late PageController _pageController;
  late MaterialLocalizations _localizations;
  late TextDirection _textDirection;
  Map<ShortcutActivator, Intent>? _shortcutMap;
  Map<Type, Action<Intent>>? _actionMap;
  late FocusNode _dayGridFocus;
  DateTime? _focusedDay;

  @override
  void initState() {
    super.initState();
    _currentMonth = widget.initialMonth;
    _pageController = PageController(
      initialPage: DateUtils.monthDelta(widget.firstDate, _currentMonth),
    );
    _shortcutMap = const <ShortcutActivator, Intent>{
      SingleActivator(LogicalKeyboardKey.arrowLeft): DirectionalFocusIntent(
        TraversalDirection.left,
      ),
      SingleActivator(LogicalKeyboardKey.arrowRight): DirectionalFocusIntent(
        TraversalDirection.right,
      ),
      SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(
        TraversalDirection.down,
      ),
      SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(TraversalDirection.up),
    };
    _actionMap = <Type, Action<Intent>>{
      NextFocusIntent: CallbackAction<NextFocusIntent>(onInvoke: _handleGridNextFocus),
      PreviousFocusIntent: CallbackAction<PreviousFocusIntent>(onInvoke: _handleGridPreviousFocus),
      DirectionalFocusIntent: CallbackAction<DirectionalFocusIntent>(
        onInvoke: _handleDirectionFocus,
      ),
    };
    _dayGridFocus = FocusNode(debugLabel: 'Day Grid');
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _localizations = MaterialLocalizations.of(context);
    _textDirection = Directionality.of(context);
  }

  @override
  void dispose() {
    _pageController.dispose();
    _dayGridFocus.dispose();
    super.dispose();
  }

  void _handleDateSelected(DateTime selectedDate) {
    _focusedDay = selectedDate;
    widget.onChanged(selectedDate);
  }

  void _handleMonthPageChanged(int monthPage) {
    setState(() {
      final DateTime monthDate = DateUtils.addMonthsToMonthDate(widget.firstDate, monthPage);
      if (!DateUtils.isSameMonth(_currentMonth, monthDate)) {
        _currentMonth = DateTime(monthDate.year, monthDate.month);
        widget.onDisplayedMonthChanged(_currentMonth);
        if (_focusedDay != null && !DateUtils.isSameMonth(_focusedDay, _currentMonth)) {
          // We have navigated to a new month with the grid focused, but the
          // focused day is not in this month. Choose a new one trying to keep
          // the same day of the month.
          _focusedDay = _focusableDayForMonth(_currentMonth, _focusedDay!.day);
        }
        SemanticsService.announce(_localizations.formatMonthYear(_currentMonth), _textDirection);
      }
    });
  }

  /// Returns a focusable date for the given month.
  ///
  /// If the preferredDay is available in the month it will be returned,
  /// otherwise the first selectable day in the month will be returned. If
  /// no dates are selectable in the month, then it will return null.
  DateTime? _focusableDayForMonth(DateTime month, int preferredDay) {
    final int daysInMonth = DateUtils.getDaysInMonth(month.year, month.month);

    // Can we use the preferred day in this month?
    if (preferredDay <= daysInMonth) {
      final DateTime newFocus = DateTime(month.year, month.month, preferredDay);
      if (_isSelectable(newFocus)) {
        return newFocus;
      }
    }

    // Start at the 1st and take the first selectable date.
    for (int day = 1; day <= daysInMonth; day++) {
      final DateTime newFocus = DateTime(month.year, month.month, day);
      if (_isSelectable(newFocus)) {
        return newFocus;
      }
    }
    return null;
  }

  /// Navigate to the next month.
  void _handleNextMonth() {
    if (!_isDisplayingLastMonth) {
      _pageController.nextPage(duration: _monthScrollDuration, curve: Curves.ease);
    }
  }

  /// Navigate to the previous month.
  void _handlePreviousMonth() {
    if (!_isDisplayingFirstMonth) {
      _pageController.previousPage(duration: _monthScrollDuration, curve: Curves.ease);
    }
  }

  /// Navigate to the given month.
  void _showMonth(DateTime month, {bool jump = false}) {
    final int monthPage = DateUtils.monthDelta(widget.firstDate, month);
    if (jump) {
      _pageController.jumpToPage(monthPage);
    } else {
      _pageController.animateToPage(monthPage, duration: _monthScrollDuration, curve: Curves.ease);
    }
  }

  /// True if the earliest allowable month is displayed.
  bool get _isDisplayingFirstMonth {
    return !_currentMonth.isAfter(DateTime(widget.firstDate.year, widget.firstDate.month));
  }

  /// True if the latest allowable month is displayed.
  bool get _isDisplayingLastMonth {
    return !_currentMonth.isBefore(DateTime(widget.lastDate.year, widget.lastDate.month));
  }

  /// Handler for when the overall day grid obtains or loses focus.
  void _handleGridFocusChange(bool focused) {
    setState(() {
      if (focused && _focusedDay == null) {
        if (DateUtils.isSameMonth(widget.selectedDate, _currentMonth)) {
          _focusedDay = widget.selectedDate;
        } else if (DateUtils.isSameMonth(widget.currentDate, _currentMonth)) {
          _focusedDay = _focusableDayForMonth(_currentMonth, widget.currentDate.day);
        } else {
          _focusedDay = _focusableDayForMonth(_currentMonth, 1);
        }
      }
    });
  }

  /// Move focus to the next element after the day grid.
  void _handleGridNextFocus(NextFocusIntent intent) {
    _dayGridFocus.requestFocus();
    _dayGridFocus.nextFocus();
  }

  /// Move focus to the previous element before the day grid.
  void _handleGridPreviousFocus(PreviousFocusIntent intent) {
    _dayGridFocus.requestFocus();
    _dayGridFocus.previousFocus();
  }

  /// Move the internal focus date in the direction of the given intent.
  ///
  /// This will attempt to move the focused day to the next selectable day in
  /// the given direction. If the new date is not in the current month, then
  /// the page view will be scrolled to show the new date's month.
  ///
  /// For horizontal directions, it will move forward or backward a day (depending
  /// on the current [TextDirection]). For vertical directions it will move up and
  /// down a week at a time.
  void _handleDirectionFocus(DirectionalFocusIntent intent) {
    assert(_focusedDay != null);
    setState(() {
      final DateTime? nextDate = _nextDateInDirection(_focusedDay!, intent.direction);
      if (nextDate != null) {
        _focusedDay = nextDate;
        if (!DateUtils.isSameMonth(_focusedDay, _currentMonth)) {
          _showMonth(_focusedDay!);
        }
      }
    });
  }

  static const Map<TraversalDirection, int> _directionOffset = <TraversalDirection, int>{
    TraversalDirection.up: -DateTime.daysPerWeek,
    TraversalDirection.right: 1,
    TraversalDirection.down: DateTime.daysPerWeek,
    TraversalDirection.left: -1,
  };

  int _dayDirectionOffset(TraversalDirection traversalDirection, TextDirection textDirection) {
    // Swap left and right if the text direction if RTL
    if (textDirection == TextDirection.rtl) {
      if (traversalDirection == TraversalDirection.left) {
        traversalDirection = TraversalDirection.right;
      } else if (traversalDirection == TraversalDirection.right) {
        traversalDirection = TraversalDirection.left;
      }
    }
    return _directionOffset[traversalDirection]!;
  }

  DateTime? _nextDateInDirection(DateTime date, TraversalDirection direction) {
    final TextDirection textDirection = Directionality.of(context);
    DateTime nextDate = DateUtils.addDaysToDate(
      date,
      _dayDirectionOffset(direction, textDirection),
    );
    while (!nextDate.isBefore(widget.firstDate) && !nextDate.isAfter(widget.lastDate)) {
      if (_isSelectable(nextDate)) {
        return nextDate;
      }
      nextDate = DateUtils.addDaysToDate(nextDate, _dayDirectionOffset(direction, textDirection));
    }
    return null;
  }

  bool _isSelectable(DateTime date) {
    return widget.selectableDayPredicate?.call(date) ?? true;
  }

  Widget _buildItems(BuildContext context, int index) {
    final DateTime month = DateUtils.addMonthsToMonthDate(widget.firstDate, index);
    return _DayPicker(
      key: ValueKey<DateTime>(month),
      selectedDate: widget.selectedDate,
      currentDate: widget.currentDate,
      onChanged: _handleDateSelected,
      firstDate: widget.firstDate,
      lastDate: widget.lastDate,
      displayedMonth: month,
      selectableDayPredicate: widget.selectableDayPredicate,
    );
  }

  @override
  Widget build(BuildContext context) {
    final Color controlColor = Theme.of(context).colorScheme.onSurface.withOpacity(0.60);

    return Semantics(
      container: true,
      explicitChildNodes: true,
      child: Column(
        children: <Widget>[
          SizedBox(
            height: _subHeaderHeight,
            child: Padding(
              padding: const EdgeInsetsDirectional.only(start: 16, end: 4),
              child: Row(
                children: <Widget>[
                  const Spacer(),
                  IconButton(
                    icon: const Icon(Icons.chevron_left),
                    color: controlColor,
                    tooltip: _isDisplayingFirstMonth ? null : _localizations.previousMonthTooltip,
                    onPressed: _isDisplayingFirstMonth ? null : _handlePreviousMonth,
                  ),
                  IconButton(
                    icon: const Icon(Icons.chevron_right),
                    color: controlColor,
                    tooltip: _isDisplayingLastMonth ? null : _localizations.nextMonthTooltip,
                    onPressed: _isDisplayingLastMonth ? null : _handleNextMonth,
                  ),
                ],
              ),
            ),
          ),
          Expanded(
            child: FocusableActionDetector(
              shortcuts: _shortcutMap,
              actions: _actionMap,
              focusNode: _dayGridFocus,
              onFocusChange: _handleGridFocusChange,
              child: _FocusedDate(
                date: _dayGridFocus.hasFocus ? _focusedDay : null,
                child: PageView.builder(
                  key: _pageViewKey,
                  controller: _pageController,
                  itemBuilder: _buildItems,
                  itemCount: DateUtils.monthDelta(widget.firstDate, widget.lastDate) + 1,
                  onPageChanged: _handleMonthPageChanged,
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

/// InheritedWidget indicating what the current focused date is for its children.
///
/// This is used by the [_MonthPicker] to let its children [_DayPicker]s know
/// what the currently focused date (if any) should be.
class _FocusedDate extends InheritedWidget {
  const _FocusedDate({required super.child, this.date});

  final DateTime? date;

  @override
  bool updateShouldNotify(_FocusedDate oldWidget) {
    return !DateUtils.isSameDay(date, oldWidget.date);
  }

  static DateTime? maybeOf(BuildContext context) {
    final _FocusedDate? focusedDate = context.dependOnInheritedWidgetOfExactType<_FocusedDate>();
    return focusedDate?.date;
  }
}

/// Displays the days of a given month and allows choosing a day.
///
/// The days are arranged in a rectangular grid with one column for each day of
/// the week.
class _DayPicker extends StatefulWidget {
  /// Creates a day picker.
  _DayPicker({
    super.key,
    required this.currentDate,
    required this.displayedMonth,
    required this.firstDate,
    required this.lastDate,
    required this.selectedDate,
    required this.onChanged,
    this.selectableDayPredicate,
  }) : assert(!firstDate.isAfter(lastDate)),
       assert(selectedDate == null || !selectedDate.isBefore(firstDate)),
       assert(selectedDate == null || !selectedDate.isAfter(lastDate));

  /// The currently selected date.
  ///
  /// This date is highlighted in the picker.
  final DateTime? selectedDate;

  /// The current date at the time the picker is displayed.
  final DateTime currentDate;

  /// Called when the user picks a day.
  final ValueChanged<DateTime> onChanged;

  /// The earliest date the user is permitted to pick.
  ///
  /// This date must be on or before the [lastDate].
  final DateTime firstDate;

  /// The latest date the user is permitted to pick.
  ///
  /// This date must be on or after the [firstDate].
  final DateTime lastDate;

  /// The month whose days are displayed by this picker.
  final DateTime displayedMonth;

  /// Optional user supplied predicate function to customize selectable days.
  final SelectableDayPredicate? selectableDayPredicate;

  @override
  _DayPickerState createState() => _DayPickerState();
}

class _DayPickerState extends State<_DayPicker> {
  /// List of [FocusNode]s, one for each day of the month.
  late List<FocusNode> _dayFocusNodes;

  @override
  void initState() {
    super.initState();
    final int daysInMonth = DateUtils.getDaysInMonth(
      widget.displayedMonth.year,
      widget.displayedMonth.month,
    );
    _dayFocusNodes = List<FocusNode>.generate(
      daysInMonth,
      (int index) => FocusNode(skipTraversal: true, debugLabel: 'Day ${index + 1}'),
    );
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // Check to see if the focused date is in this month, if so focus it.
    final DateTime? focusedDate = _FocusedDate.maybeOf(context);
    if (focusedDate != null && DateUtils.isSameMonth(widget.displayedMonth, focusedDate)) {
      _dayFocusNodes[focusedDate.day - 1].requestFocus();
    }
  }

  @override
  void dispose() {
    for (final FocusNode node in _dayFocusNodes) {
      node.dispose();
    }
    super.dispose();
  }

  /// Builds widgets showing abbreviated days of week. The first widget in the
  /// returned list corresponds to the first day of week for the current locale.
  ///
  /// Examples:
  ///
  ///     ┌ Sunday is the first day of week in the US (en_US)
  ///     |
  ///     S M T W T F S  ← the returned list contains these widgets
  ///     _ _ _ _ _ 1 2
  ///     3 4 5 6 7 8 9
  ///
  ///     ┌ But it's Monday in the UK (en_GB)
  ///     |
  ///     M T W T F S S  ← the returned list contains these widgets
  ///     _ _ _ _ 1 2 3
  ///     4 5 6 7 8 9 10
  ///
  List<Widget> _dayHeaders(TextStyle? headerStyle, MaterialLocalizations localizations) {
    final List<Widget> result = <Widget>[];
    for (
      int i = localizations.firstDayOfWeekIndex;
      result.length < DateTime.daysPerWeek;
      i = (i + 1) % DateTime.daysPerWeek
    ) {
      final String weekday = localizations.narrowWeekdays[i];
      result.add(ExcludeSemantics(child: Center(child: Text(weekday, style: headerStyle))));
    }
    return result;
  }

  @override
  Widget build(BuildContext context) {
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);
    final TextStyle? weekdayStyle = datePickerTheme.weekdayStyle ?? defaults.weekdayStyle;

    final Orientation orientation = MediaQuery.orientationOf(context);
    final bool isLandscapeOrientation = orientation == Orientation.landscape;

    final int year = widget.displayedMonth.year;
    final int month = widget.displayedMonth.month;

    final int daysInMonth = DateUtils.getDaysInMonth(year, month);
    final int dayOffset = DateUtils.firstDayOffset(year, month, localizations);

    final List<Widget> dayItems = _dayHeaders(weekdayStyle, localizations);
    // 1-based day of month, e.g. 1-31 for January, and 1-29 for February on
    // a leap year.
    int day = -dayOffset;
    while (day < daysInMonth) {
      day++;
      if (day < 1) {
        dayItems.add(const SizedBox.shrink());
      } else {
        final DateTime dayToBuild = DateTime(year, month, day);
        final bool isDisabled =
            dayToBuild.isAfter(widget.lastDate) ||
            dayToBuild.isBefore(widget.firstDate) ||
            (widget.selectableDayPredicate != null && !widget.selectableDayPredicate!(dayToBuild));
        final bool isSelectedDay = DateUtils.isSameDay(widget.selectedDate, dayToBuild);
        final bool isToday = DateUtils.isSameDay(widget.currentDate, dayToBuild);

        dayItems.add(
          _Day(
            dayToBuild,
            key: ValueKey<DateTime>(dayToBuild),
            isDisabled: isDisabled,
            isSelectedDay: isSelectedDay,
            isToday: isToday,
            onChanged: widget.onChanged,
            focusNode: _dayFocusNodes[day - 1],
          ),
        );
      }
    }

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: _monthPickerHorizontalPadding),
      child: MediaQuery.withClampedTextScaling(
        maxScaleFactor:
            isLandscapeOrientation
                ? _kDayPickerGridLandscapeMaxScaleFactor
                : _kDayPickerGridPortraitMaxScaleFactor,
        child: GridView.custom(
          physics: const ClampingScrollPhysics(),
          gridDelegate: _DayPickerGridDelegate(context),
          childrenDelegate: SliverChildListDelegate(dayItems, addRepaintBoundaries: false),
        ),
      ),
    );
  }
}

class _Day extends StatefulWidget {
  const _Day(
    this.day, {
    super.key,
    required this.isDisabled,
    required this.isSelectedDay,
    required this.isToday,
    required this.onChanged,
    required this.focusNode,
  });

  final DateTime day;
  final bool isDisabled;
  final bool isSelectedDay;
  final bool isToday;
  final ValueChanged<DateTime> onChanged;
  final FocusNode focusNode;

  @override
  State<_Day> createState() => _DayState();
}

class _DayState extends State<_Day> {
  final MaterialStatesController _statesController = MaterialStatesController();

  @override
  Widget build(BuildContext context) {
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    final TextStyle? dayStyle = datePickerTheme.dayStyle ?? defaults.dayStyle;
    T? effectiveValue<T>(T? Function(DatePickerThemeData? theme) getProperty) {
      return getProperty(datePickerTheme) ?? getProperty(defaults);
    }

    T? resolve<T>(
      MaterialStateProperty<T>? Function(DatePickerThemeData? theme) getProperty,
      Set<MaterialState> states,
    ) {
      return effectiveValue((DatePickerThemeData? theme) {
        return getProperty(theme)?.resolve(states);
      });
    }

    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final String semanticLabelSuffix = widget.isToday ? ', ${localizations.currentDateLabel}' : '';

    final Set<MaterialState> states = <MaterialState>{
      if (widget.isDisabled) MaterialState.disabled,
      if (widget.isSelectedDay) MaterialState.selected,
    };

    _statesController.value = states;

    final Color? dayForegroundColor = resolve<Color?>(
      (DatePickerThemeData? theme) =>
          widget.isToday ? theme?.todayForegroundColor : theme?.dayForegroundColor,
      states,
    );
    final Color? dayBackgroundColor = resolve<Color?>(
      (DatePickerThemeData? theme) =>
          widget.isToday ? theme?.todayBackgroundColor : theme?.dayBackgroundColor,
      states,
    );
    final MaterialStateProperty<Color?> dayOverlayColor = MaterialStateProperty.resolveWith<Color?>(
      (Set<MaterialState> states) =>
          effectiveValue((DatePickerThemeData? theme) => theme?.dayOverlayColor?.resolve(states)),
    );
    final OutlinedBorder dayShape =
        resolve<OutlinedBorder?>((DatePickerThemeData? theme) => theme?.dayShape, states)!;
    final ShapeDecoration decoration =
        widget.isToday
            ? ShapeDecoration(
              color: dayBackgroundColor,
              shape: dayShape.copyWith(
                side: (datePickerTheme.todayBorder ?? defaults.todayBorder!).copyWith(
                  color: dayForegroundColor,
                ),
              ),
            )
            : ShapeDecoration(color: dayBackgroundColor, shape: dayShape);

    Widget dayWidget = Ink(
      decoration: decoration,
      child: Center(
        child: Text(
          localizations.formatDecimal(widget.day.day),
          style: dayStyle?.apply(color: dayForegroundColor),
        ),
      ),
    );

    if (widget.isDisabled) {
      dayWidget = ExcludeSemantics(child: dayWidget);
    } else {
      dayWidget = InkResponse(
        focusNode: widget.focusNode,
        onTap: () => widget.onChanged(widget.day),
        statesController: _statesController,
        overlayColor: dayOverlayColor,
        customBorder: dayShape,
        containedInkWell: true,
        child: Semantics(
          // We want the day of month to be spoken first irrespective of the
          // locale-specific preferences or TextDirection. This is because
          // an accessibility user is more likely to be interested in the
          // day of month before the rest of the date, as they are looking
          // for the day of month. To do that we prepend day of month to the
          // formatted full date.
          label:
              '${localizations.formatDecimal(widget.day.day)}, ${localizations.formatFullDate(widget.day)}$semanticLabelSuffix',
          // Set button to true to make the date selectable.
          button: true,
          selected: widget.isSelectedDay,
          excludeSemantics: true,
          child: dayWidget,
        ),
      );
    }

    return dayWidget;
  }

  @override
  void dispose() {
    _statesController.dispose();
    super.dispose();
  }
}

class _DayPickerGridDelegate extends SliverGridDelegate {
  const _DayPickerGridDelegate(this.context);

  final BuildContext context;

  @override
  SliverGridLayout getLayout(SliverConstraints constraints) {
    final double textScaleFactor =
        MediaQuery.textScalerOf(context).clamp(maxScaleFactor: 3.0).scale(_fontSizeToScale) /
        _fontSizeToScale;
    final double scaledRowHeight =
        textScaleFactor > 1.3
            ? ((textScaleFactor - 1) * 30) + _dayPickerRowHeight
            : _dayPickerRowHeight;
    const int columnCount = DateTime.daysPerWeek;
    final double tileWidth = constraints.crossAxisExtent / columnCount;
    final double tileHeight = math.min(
      scaledRowHeight,
      constraints.viewportMainAxisExtent / (_maxDayPickerRowCount + 1),
    );
    return SliverGridRegularTileLayout(
      childCrossAxisExtent: tileWidth,
      childMainAxisExtent: tileHeight,
      crossAxisCount: columnCount,
      crossAxisStride: tileWidth,
      mainAxisStride: tileHeight,
      reverseCrossAxis: axisDirectionIsReversed(constraints.crossAxisDirection),
    );
  }

  @override
  bool shouldRelayout(_DayPickerGridDelegate oldDelegate) => false;
}

/// A scrollable grid of years to allow picking a year.
///
/// The year picker widget is rarely used directly. Instead, consider using
/// [CalendarDatePicker], or [showDatePicker] which create full date pickers.
///
/// See also:
///
///  * [CalendarDatePicker], which provides a Material Design date picker
///    interface.
///
///  * [showDatePicker], which shows a dialog containing a Material Design
///    date picker.
///
class YearPicker extends StatefulWidget {
  /// Creates a year picker.
  ///
  /// The [lastDate] must be after the [firstDate].
  YearPicker({
    super.key,
    DateTime? currentDate,
    required this.firstDate,
    required this.lastDate,
    @Deprecated(
      'This parameter has no effect and can be removed. Previously it controlled '
      'the month that was used in "onChanged" when a new year was selected, but '
      'now that role is filled by "selectedDate" instead. '
      'This feature was deprecated after v3.13.0-0.3.pre.',
    )
    DateTime? initialDate,
    required this.selectedDate,
    required this.onChanged,
    this.dragStartBehavior = DragStartBehavior.start,
  }) : assert(!firstDate.isAfter(lastDate)),
       currentDate = DateUtils.dateOnly(currentDate ?? DateTime.now());

  /// The current date.
  ///
  /// This date is subtly highlighted in the picker.
  final DateTime currentDate;

  /// The earliest date the user is permitted to pick.
  final DateTime firstDate;

  /// The latest date the user is permitted to pick.
  final DateTime lastDate;

  /// The currently selected date.
  ///
  /// This date is highlighted in the picker.
  final DateTime? selectedDate;

  /// Called when the user picks a year.
  final ValueChanged<DateTime> onChanged;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  @override
  State<YearPicker> createState() => _YearPickerState();
}

class _YearPickerState extends State<YearPicker> {
  ScrollController? _scrollController;
  final MaterialStatesController _statesController = MaterialStatesController();

  // The approximate number of years necessary to fill the available space.
  static const int minYears = 18;

  @override
  void initState() {
    super.initState();
    _scrollController = ScrollController(
      initialScrollOffset: _scrollOffsetForYear(widget.selectedDate ?? widget.firstDate),
    );
  }

  @override
  void dispose() {
    _scrollController?.dispose();
    _statesController.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(YearPicker oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.selectedDate != oldWidget.selectedDate && widget.selectedDate != null) {
      _scrollController!.jumpTo(_scrollOffsetForYear(widget.selectedDate!));
    }
  }

  double _scrollOffsetForYear(DateTime date) {
    final int initialYearIndex = date.year - widget.firstDate.year;
    final int initialYearRow = initialYearIndex ~/ _yearPickerColumnCount;
    // Move the offset down by 2 rows to approximately center it.
    final int centeredYearRow = initialYearRow - 2;
    return _itemCount < minYears ? 0 : centeredYearRow * _yearPickerRowHeight;
  }

  Widget _buildYearItem(BuildContext context, int index) {
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);

    T? effectiveValue<T>(T? Function(DatePickerThemeData? theme) getProperty) {
      return getProperty(datePickerTheme) ?? getProperty(defaults);
    }

    T? resolve<T>(
      MaterialStateProperty<T>? Function(DatePickerThemeData? theme) getProperty,
      Set<MaterialState> states,
    ) {
      return effectiveValue((DatePickerThemeData? theme) {
        return getProperty(theme)?.resolve(states);
      });
    }

    final double textScaleFactor =
        MediaQuery.textScalerOf(context).clamp(maxScaleFactor: 3.0).scale(_fontSizeToScale) /
        _fontSizeToScale;

    // Backfill the _YearPicker with disabled years if necessary.
    final int offset = _itemCount < minYears ? (minYears - _itemCount) ~/ 2 : 0;
    final int year = widget.firstDate.year + index - offset;
    final bool isSelected = year == widget.selectedDate?.year;
    final bool isCurrentYear = year == widget.currentDate.year;
    final bool isDisabled = year < widget.firstDate.year || year > widget.lastDate.year;
    final double decorationHeight = 36.0 * textScaleFactor;
    final double decorationWidth = 72.0 * textScaleFactor;

    final Set<MaterialState> states = <MaterialState>{
      if (isDisabled) MaterialState.disabled,
      if (isSelected) MaterialState.selected,
    };

    final Color? textColor = resolve<Color?>(
      (DatePickerThemeData? theme) =>
          isCurrentYear ? theme?.todayForegroundColor : theme?.yearForegroundColor,
      states,
    );
    final Color? background = resolve<Color?>(
      (DatePickerThemeData? theme) =>
          isCurrentYear ? theme?.todayBackgroundColor : theme?.yearBackgroundColor,
      states,
    );
    final MaterialStateProperty<Color?> overlayColor = MaterialStateProperty.resolveWith<Color?>(
      (Set<MaterialState> states) =>
          effectiveValue((DatePickerThemeData? theme) => theme?.yearOverlayColor?.resolve(states)),
    );

    BoxBorder? border;
    if (isCurrentYear) {
      final BorderSide? todayBorder = datePickerTheme.todayBorder ?? defaults.todayBorder;
      if (todayBorder != null) {
        border = Border.fromBorderSide(todayBorder.copyWith(color: textColor));
      }
    }
    final BoxDecoration decoration = BoxDecoration(
      border: border,
      color: background,
      borderRadius: BorderRadius.circular(decorationHeight / 2),
    );

    final TextStyle? itemStyle = (datePickerTheme.yearStyle ?? defaults.yearStyle)?.apply(
      color: textColor,
    );
    Widget yearItem = Center(
      child: Container(
        decoration: decoration,
        height: decorationHeight,
        width: decorationWidth,
        alignment: Alignment.center,
        child: Semantics(
          selected: isSelected,
          button: true,
          child: Text(year.toString(), style: itemStyle),
        ),
      ),
    );

    if (isDisabled) {
      yearItem = ExcludeSemantics(child: yearItem);
    } else {
      DateTime date = DateTime(year, widget.selectedDate?.month ?? DateTime.january);
      if (date.isBefore(DateTime(widget.firstDate.year, widget.firstDate.month))) {
        // Ignore firstDate.day because we're just working in years and months here.
        assert(date.year == widget.firstDate.year);
        date = DateTime(year, widget.firstDate.month);
      } else if (date.isAfter(widget.lastDate)) {
        // No need to ignore the day here because it can only be bigger than what we care about.
        assert(date.year == widget.lastDate.year);
        date = DateTime(year, widget.lastDate.month);
      }
      _statesController.value = states;
      yearItem = InkWell(
        key: ValueKey<int>(year),
        onTap: () => widget.onChanged(date),
        statesController: _statesController,
        overlayColor: overlayColor,
        child: yearItem,
      );
    }

    return yearItem;
  }

  int get _itemCount {
    return widget.lastDate.year - widget.firstDate.year + 1;
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    return Column(
      children: <Widget>[
        const Divider(),
        Expanded(
          child: GridView.builder(
            controller: _scrollController,
            dragStartBehavior: widget.dragStartBehavior,
            gridDelegate: _YearPickerGridDelegate(context),
            itemBuilder: _buildYearItem,
            itemCount: math.max(_itemCount, minYears),
            padding: const EdgeInsets.symmetric(horizontal: _yearPickerPadding),
          ),
        ),
        const Divider(),
      ],
    );
  }
}

class _YearPickerGridDelegate extends SliverGridDelegate {
  const _YearPickerGridDelegate(this.context);

  final BuildContext context;

  @override
  SliverGridLayout getLayout(SliverConstraints constraints) {
    final double textScaleFactor =
        MediaQuery.textScalerOf(context).clamp(maxScaleFactor: 3.0).scale(_fontSizeToScale) /
        _fontSizeToScale;
    final int scaledYearPickerColumnCount =
        textScaleFactor > 1.65 ? _yearPickerColumnCount - 1 : _yearPickerColumnCount;
    final double tileWidth =
        (constraints.crossAxisExtent - (scaledYearPickerColumnCount - 1) * _yearPickerRowSpacing) /
        scaledYearPickerColumnCount;
    final double scaledYearPickerRowHeight =
        textScaleFactor > 1
            ? _yearPickerRowHeight + ((textScaleFactor - 1) * 9)
            : _yearPickerRowHeight;
    return SliverGridRegularTileLayout(
      childCrossAxisExtent: tileWidth,
      childMainAxisExtent: scaledYearPickerRowHeight,
      crossAxisCount: scaledYearPickerColumnCount,
      crossAxisStride: tileWidth + _yearPickerRowSpacing,
      mainAxisStride: scaledYearPickerRowHeight,
      reverseCrossAxis: axisDirectionIsReversed(constraints.crossAxisDirection),
    );
  }

  @override
  bool shouldRelayout(_YearPickerGridDelegate oldDelegate) => false;
}
