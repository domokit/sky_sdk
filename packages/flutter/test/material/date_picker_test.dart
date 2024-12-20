// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/clipboard_utils.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  late DateTime firstDate;
  late DateTime lastDate;
  late DateTime? initialDate;
  late DateTime today;
  late SelectableDayPredicate? selectableDayPredicate;
  late DatePickerEntryMode initialEntryMode;
  late DatePickerMode initialCalendarMode;
  late DatePickerEntryMode currentMode;

  String? cancelText;
  String? confirmText;
  String? errorFormatText;
  String? errorInvalidText;
  String? fieldHintText;
  String? fieldLabelText;
  String? helpText;
  TextInputType? keyboardType;

  final Finder nextMonthIcon = find.byWidgetPredicate(
    (Widget w) => w is IconButton && (w.tooltip?.startsWith('Next month') ?? false),
  );
  final Finder previousMonthIcon = find.byWidgetPredicate(
    (Widget w) => w is IconButton && (w.tooltip?.startsWith('Previous month') ?? false),
  );
  final Finder switchToInputIcon = find.byIcon(Icons.edit);
  final Finder switchToCalendarIcon = find.byIcon(Icons.calendar_today);

  TextField textField(WidgetTester tester) {
    return tester.widget<TextField>(find.byType(TextField));
  }

  setUp(() {
    firstDate = DateTime(2001);
    lastDate = DateTime(2031, DateTime.december, 31);
    initialDate = DateTime(2016, DateTime.january, 15);
    today = DateTime(2016, DateTime.january, 3);
    selectableDayPredicate = null;
    initialEntryMode = DatePickerEntryMode.calendar;
    initialCalendarMode = DatePickerMode.day;

    cancelText = null;
    confirmText = null;
    errorFormatText = null;
    errorInvalidText = null;
    fieldHintText = null;
    fieldLabelText = null;
    helpText = null;
    keyboardType = null;
    currentMode = initialEntryMode;
  });

  const Size wideWindowSize = Size(1920.0, 1080.0);
  const Size narrowWindowSize = Size(1070.0, 1770.0);

  Future<void> prepareDatePicker(
    WidgetTester tester,
    Future<void> Function(Future<DateTime?> date) callback, {
    TextDirection textDirection = TextDirection.ltr,
    bool useMaterial3 = false,
    ThemeData? theme,
    TextScaler textScaler = TextScaler.noScaling,
  }) async {
    late BuildContext buttonContext;
    await tester.pumpWidget(
      MaterialApp(
        theme: theme ?? ThemeData(useMaterial3: useMaterial3),
        home: MediaQuery(
          data: MediaQueryData(textScaler: textScaler),
          child: Material(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  onPressed: () {
                    buttonContext = context;
                  },
                  child: const Text('Go'),
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Go'));
    expect(buttonContext, isNotNull);

    final Future<DateTime?> date = showDatePicker(
      context: buttonContext,
      initialDate: initialDate,
      firstDate: firstDate,
      lastDate: lastDate,
      currentDate: today,
      selectableDayPredicate: selectableDayPredicate,
      initialDatePickerMode: initialCalendarMode,
      initialEntryMode: initialEntryMode,
      cancelText: cancelText,
      confirmText: confirmText,
      errorFormatText: errorFormatText,
      errorInvalidText: errorInvalidText,
      fieldHintText: fieldHintText,
      fieldLabelText: fieldLabelText,
      helpText: helpText,
      keyboardType: keyboardType,
      onDatePickerModeChange: (DatePickerEntryMode value) {
        currentMode = value;
      },
      builder: (BuildContext context, Widget? child) {
        return Directionality(textDirection: textDirection, child: child ?? const SizedBox());
      },
    );

    await tester.pumpAndSettle(const Duration(seconds: 1));
    await callback(date);
  }

  ShapeDecoration? findDayDecoration(WidgetTester tester, String day) {
    return tester
            .widget<Ink>(find.ancestor(of: find.text(day), matching: find.byType(Ink)))
            .decoration
        as ShapeDecoration?;
  }

  MaterialInkController findDayGridMaterial(WidgetTester tester) {
    // All days are painted on the same Material widget.
    // Use an arbitrary day to find this Material.
    return Material.of(tester.element(find.text('17')));
  }

  group('showDatePicker Dialog', () {
    testWidgets('Default dialog size', (WidgetTester tester) async {
      Future<void> showPicker(WidgetTester tester, Size size) async {
        tester.view.physicalSize = size;
        tester.view.devicePixelRatio = 1.0;
        addTearDown(tester.view.reset);
        await prepareDatePicker(tester, (Future<DateTime?> date) async {}, useMaterial3: true);
      }

      const Size calendarLandscapeDialogSize = Size(496.0, 346.0);
      const Size calendarPortraitDialogSizeM3 = Size(328.0, 512.0);

      // Test landscape layout.
      await showPicker(tester, wideWindowSize);

      Size dialogContainerSize = tester.getSize(find.byType(AnimatedContainer));
      expect(dialogContainerSize, calendarLandscapeDialogSize);

      // Close the dialog.
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();

      // Test portrait layout.
      await showPicker(tester, narrowWindowSize);

      dialogContainerSize = tester.getSize(find.byType(AnimatedContainer));
      expect(dialogContainerSize, calendarPortraitDialogSizeM3);
    });

    testWidgets('Default dialog properties', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: true);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final Material dialogMaterial = tester.widget<Material>(
          find.descendant(of: find.byType(Dialog), matching: find.byType(Material)).first,
        );

        expect(dialogMaterial.color, theme.colorScheme.surfaceContainerHigh);
        expect(dialogMaterial.shadowColor, Colors.transparent);
        expect(dialogMaterial.surfaceTintColor, Colors.transparent);
        expect(dialogMaterial.elevation, 6.0);
        expect(
          dialogMaterial.shape,
          const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(28.0))),
        );
        expect(dialogMaterial.clipBehavior, Clip.antiAlias);

        final Dialog dialog = tester.widget<Dialog>(find.byType(Dialog));
        expect(dialog.insetPadding, const EdgeInsets.symmetric(horizontal: 16.0, vertical: 24.0));
      }, useMaterial3: theme.useMaterial3);
    });

    testWidgets('Material3 uses sentence case labels', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.text('Select date'), findsOneWidget);
      }, useMaterial3: true);
    });

    testWidgets('Cancel, confirm, and help text is used', (WidgetTester tester) async {
      cancelText = 'nope';
      confirmText = 'yep';
      helpText = 'help';
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.text(cancelText!), findsOneWidget);
        expect(find.text(confirmText!), findsOneWidget);
        expect(find.text(helpText!), findsOneWidget);
      });
    });

    testWidgets('Initial date is the default', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2016, DateTime.january, 15));
      });
    });

    testWidgets('Can cancel', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('CANCEL'));
        expect(await date, isNull);
      });
    });

    testWidgets('Can switch from calendar to input entry mode', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.byType(TextField), findsNothing);
        await tester.tap(find.byIcon(Icons.edit));
        await tester.pumpAndSettle();
        expect(find.byType(TextField), findsOneWidget);
      });
    });

    testWidgets('Can switch from input to calendar entry mode', (WidgetTester tester) async {
      initialEntryMode = DatePickerEntryMode.input;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.byType(TextField), findsOneWidget);
        await tester.tap(find.byIcon(Icons.calendar_today));
        await tester.pumpAndSettle();
        expect(find.byType(TextField), findsNothing);
      });
    });

    testWidgets('Can not switch out of calendarOnly mode', (WidgetTester tester) async {
      initialEntryMode = DatePickerEntryMode.calendarOnly;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.byType(TextField), findsNothing);
        expect(find.byIcon(Icons.edit), findsNothing);
      });
    });

    testWidgets('Can not switch out of inputOnly mode', (WidgetTester tester) async {
      initialEntryMode = DatePickerEntryMode.inputOnly;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.byType(TextField), findsOneWidget);
        expect(find.byIcon(Icons.calendar_today), findsNothing);
      });
    });

    testWidgets('Switching to input mode keeps selected date', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('12'));
        await tester.tap(find.byIcon(Icons.edit));
        await tester.pumpAndSettle();
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2016, DateTime.january, 12));
      });
    });

    testWidgets('Input only mode should validate date', (WidgetTester tester) async {
      initialEntryMode = DatePickerEntryMode.inputOnly;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Enter text input mode and type an invalid date to get error.
        await tester.enterText(find.byType(TextField), '1234567');
        await tester.tap(find.text('OK'));
        await tester.pumpAndSettle();
        expect(find.text('Invalid format.'), findsOneWidget);
      });
    });

    testWidgets('Switching to input mode resets input error state', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Enter text input mode and type an invalid date to get error.
        await tester.tap(find.byIcon(Icons.edit));
        await tester.pumpAndSettle();
        await tester.enterText(find.byType(TextField), '1234567');
        await tester.tap(find.text('OK'));
        await tester.pumpAndSettle();
        expect(find.text('Invalid format.'), findsOneWidget);

        // Toggle to calendar mode and then back to input mode
        await tester.tap(find.byIcon(Icons.calendar_today));
        await tester.pumpAndSettle();
        await tester.tap(find.byIcon(Icons.edit));
        await tester.pumpAndSettle();
        expect(find.text('Invalid format.'), findsNothing);

        // Edit the text, the error should not be showing until ok is tapped
        await tester.enterText(find.byType(TextField), '1234567');
        await tester.pumpAndSettle();
        expect(find.text('Invalid format.'), findsNothing);
      });
    });

    testWidgets('builder parameter', (WidgetTester tester) async {
      Widget buildFrame(TextDirection textDirection) {
        return MaterialApp(
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    child: const Text('X'),
                    onPressed: () {
                      showDatePicker(
                        context: context,
                        initialDate: DateTime.now(),
                        firstDate: DateTime(2018),
                        lastDate: DateTime(2030),
                        builder: (BuildContext context, Widget? child) {
                          return Directionality(
                            textDirection: textDirection,
                            child: child ?? const SizedBox(),
                          );
                        },
                      );
                    },
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(TextDirection.ltr));
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle();
      final double ltrOkRight = tester.getBottomRight(find.text('OK')).dx;

      await tester.tap(find.text('OK')); // Dismiss the dialog.
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildFrame(TextDirection.rtl));
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle();

      // Verify that the time picker is being laid out RTL.
      // We expect the left edge of the 'OK' button in the RTL
      // layout to match the gap between right edge of the 'OK'
      // button and the right edge of the 800 wide view.
      expect(tester.getBottomLeft(find.text('OK')).dx, moreOrLessEquals(800 - ltrOkRight));
    });

    group('Barrier dismissible', () {
      late _DatePickerObserver rootObserver;

      setUp(() {
        rootObserver = _DatePickerObserver();
      });

      testWidgets('Barrier is dismissible with default parameter', (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            navigatorObservers: <NavigatorObserver>[rootObserver],
            home: Material(
              child: Center(
                child: Builder(
                  builder: (BuildContext context) {
                    return ElevatedButton(
                      child: const Text('X'),
                      onPressed:
                          () => showDatePicker(
                            context: context,
                            initialDate: DateTime.now(),
                            firstDate: DateTime(2018),
                            lastDate: DateTime(2030),
                            builder: (BuildContext context, Widget? child) => const SizedBox(),
                          ),
                    );
                  },
                ),
              ),
            ),
          ),
        );

        // Open the dialog.
        await tester.tap(find.byType(ElevatedButton));
        await tester.pumpAndSettle();
        expect(rootObserver.datePickerCount, 1);

        // Tap on the barrier.
        await tester.tapAt(const Offset(10.0, 10.0));
        await tester.pumpAndSettle();
        expect(rootObserver.datePickerCount, 0);
      });

      testWidgets('Barrier is not dismissible with barrierDismissible is false', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(
          MaterialApp(
            navigatorObservers: <NavigatorObserver>[rootObserver],
            home: Material(
              child: Center(
                child: Builder(
                  builder: (BuildContext context) {
                    return ElevatedButton(
                      child: const Text('X'),
                      onPressed:
                          () => showDatePicker(
                            context: context,
                            initialDate: DateTime.now(),
                            firstDate: DateTime(2018),
                            lastDate: DateTime(2030),
                            barrierDismissible: false,
                            builder: (BuildContext context, Widget? child) => const SizedBox(),
                          ),
                    );
                  },
                ),
              ),
            ),
          ),
        );

        // Open the dialog.
        await tester.tap(find.byType(ElevatedButton));
        await tester.pumpAndSettle();
        expect(rootObserver.datePickerCount, 1);

        // Tap on the barrier, which shouldn't do anything this time.
        await tester.tapAt(const Offset(10.0, 10.0));
        await tester.pumpAndSettle();
        expect(rootObserver.datePickerCount, 1);
      });
    });

    testWidgets('Barrier color', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    child: const Text('X'),
                    onPressed:
                        () => showDatePicker(
                          context: context,
                          initialDate: DateTime.now(),
                          firstDate: DateTime(2018),
                          lastDate: DateTime(2030),
                          builder: (BuildContext context, Widget? child) => const SizedBox(),
                        ),
                  );
                },
              ),
            ),
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, Colors.black54);

      // Dismiss the dialog.
      await tester.tapAt(const Offset(10.0, 10.0));

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    child: const Text('X'),
                    onPressed:
                        () => showDatePicker(
                          context: context,
                          barrierColor: Colors.pink,
                          initialDate: DateTime.now(),
                          firstDate: DateTime(2018),
                          lastDate: DateTime(2030),
                          builder: (BuildContext context, Widget? child) => const SizedBox(),
                        ),
                  );
                },
              ),
            ),
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, Colors.pink);
    });

    testWidgets('Barrier Label', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    child: const Text('X'),
                    onPressed:
                        () => showDatePicker(
                          context: context,
                          barrierLabel: 'Custom Label',
                          initialDate: DateTime.now(),
                          firstDate: DateTime(2018),
                          lastDate: DateTime(2030),
                          builder: (BuildContext context, Widget? child) => const SizedBox(),
                        ),
                  );
                },
              ),
            ),
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(
        tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).semanticsLabel,
        'Custom Label',
      );
    });

    testWidgets('uses nested navigator if useRootNavigator is false', (WidgetTester tester) async {
      final _DatePickerObserver rootObserver = _DatePickerObserver();
      final _DatePickerObserver nestedObserver = _DatePickerObserver();

      await tester.pumpWidget(
        MaterialApp(
          navigatorObservers: <NavigatorObserver>[rootObserver],
          home: Navigator(
            observers: <NavigatorObserver>[nestedObserver],
            onGenerateRoute: (RouteSettings settings) {
              return MaterialPageRoute<dynamic>(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    onPressed: () {
                      showDatePicker(
                        context: context,
                        useRootNavigator: false,
                        initialDate: DateTime.now(),
                        firstDate: DateTime(2018),
                        lastDate: DateTime(2030),
                        builder: (BuildContext context, Widget? child) => const SizedBox(),
                      );
                    },
                    child: const Text('Show Date Picker'),
                  );
                },
              );
            },
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));

      expect(rootObserver.datePickerCount, 0);
      expect(nestedObserver.datePickerCount, 1);
    });

    testWidgets('honors DialogTheme for shape and elevation', (WidgetTester tester) async {
      // Test that the defaults work
      const DialogTheme datePickerDefaultDialogTheme = DialogTheme(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
        elevation: 24,
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('X'),
                  onPressed: () {
                    showDatePicker(
                      context: context,
                      initialDate: DateTime.now(),
                      firstDate: DateTime(2018),
                      lastDate: DateTime(2030),
                    );
                  },
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle();
      final Material defaultDialogMaterial = tester.widget<Material>(
        find.descendant(of: find.byType(Dialog), matching: find.byType(Material)).first,
      );
      expect(defaultDialogMaterial.shape, datePickerDefaultDialogTheme.shape);
      expect(defaultDialogMaterial.elevation, datePickerDefaultDialogTheme.elevation);

      // Test that it honors ThemeData.dialogTheme settings
      const DialogThemeData customDialogTheme = DialogThemeData(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(40.0))),
        elevation: 50,
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.fallback(useMaterial3: false).copyWith(dialogTheme: customDialogTheme),
          home: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('X'),
                  onPressed: () {
                    showDatePicker(
                      context: context,
                      initialDate: DateTime.now(),
                      firstDate: DateTime(2018),
                      lastDate: DateTime(2030),
                    );
                  },
                );
              },
            ),
          ),
        ),
      );
      await tester.pump(); // start theme animation
      await tester.pump(const Duration(seconds: 5)); // end theme animation
      final Material themeDialogMaterial = tester.widget<Material>(
        find.descendant(of: find.byType(Dialog), matching: find.byType(Material)).first,
      );
      expect(themeDialogMaterial.shape, customDialogTheme.shape);
      expect(themeDialogMaterial.elevation, customDialogTheme.elevation);
    });

    testWidgets('OK Cancel button layout', (WidgetTester tester) async {
      Widget buildFrame(TextDirection textDirection) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    child: const Text('X'),
                    onPressed: () {
                      showDatePicker(
                        context: context,
                        initialDate: DateTime(2016, DateTime.january, 15),
                        firstDate: DateTime(2001),
                        lastDate: DateTime(2031, DateTime.december, 31),
                        builder: (BuildContext context, Widget? child) {
                          return Directionality(
                            textDirection: textDirection,
                            child: child ?? const SizedBox(),
                          );
                        },
                      );
                    },
                  );
                },
              ),
            ),
          ),
        );
      }

      // Default landscape layout.

      await tester.pumpWidget(buildFrame(TextDirection.ltr));
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle();
      expect(tester.getBottomRight(find.text('OK')).dx, 622);
      expect(tester.getBottomLeft(find.text('OK')).dx, 594);
      expect(tester.getBottomRight(find.text('CANCEL')).dx, 560);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildFrame(TextDirection.rtl));
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle();
      expect(tester.getBottomRight(find.text('OK')).dx, 206);
      expect(tester.getBottomLeft(find.text('OK')).dx, 178);
      expect(tester.getBottomRight(find.text('CANCEL')).dx, 324);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();

      // Portrait layout.

      addTearDown(tester.view.reset);
      tester.view.physicalSize = const Size(900, 1200);

      await tester.pumpWidget(buildFrame(TextDirection.ltr));
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle();
      expect(tester.getBottomRight(find.text('OK')).dx, 258);
      expect(tester.getBottomLeft(find.text('OK')).dx, 230);
      expect(tester.getBottomRight(find.text('CANCEL')).dx, 196);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildFrame(TextDirection.rtl));
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle();
      expect(tester.getBottomRight(find.text('OK')).dx, 70);
      expect(tester.getBottomLeft(find.text('OK')).dx, 42);
      expect(tester.getBottomRight(find.text('CANCEL')).dx, 188);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();
    });

    testWidgets('honors switchToInputEntryModeIcon', (WidgetTester tester) async {
      Widget buildApp({bool? useMaterial3, Icon? switchToInputEntryModeIcon}) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: useMaterial3 ?? false),
          home: Material(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('Click X'),
                  onPressed: () {
                    showDatePicker(
                      context: context,
                      initialDate: DateTime.now(),
                      firstDate: DateTime(2018),
                      lastDate: DateTime(2030),
                      switchToInputEntryModeIcon: switchToInputEntryModeIcon,
                    );
                  },
                );
              },
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.pumpAndSettle();
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(find.byIcon(Icons.edit), findsOneWidget);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildApp(useMaterial3: true));
      await tester.pumpAndSettle();
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(find.byIcon(Icons.edit_outlined), findsOneWidget);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildApp(switchToInputEntryModeIcon: const Icon(Icons.keyboard)));
      await tester.pumpAndSettle();
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(find.byIcon(Icons.keyboard), findsOneWidget);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();
    });

    testWidgets('honors switchToCalendarEntryModeIcon', (WidgetTester tester) async {
      Widget buildApp({bool? useMaterial3, Icon? switchToCalendarEntryModeIcon}) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: useMaterial3 ?? false),
          home: Material(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('Click X'),
                  onPressed: () {
                    showDatePicker(
                      context: context,
                      initialDate: DateTime.now(),
                      firstDate: DateTime(2018),
                      lastDate: DateTime(2030),
                      switchToCalendarEntryModeIcon: switchToCalendarEntryModeIcon,
                      initialEntryMode: DatePickerEntryMode.input,
                    );
                  },
                );
              },
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.pumpAndSettle();
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(find.byIcon(Icons.calendar_today), findsOneWidget);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildApp(useMaterial3: true));
      await tester.pumpAndSettle();
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(find.byIcon(Icons.calendar_today), findsOneWidget);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildApp(switchToCalendarEntryModeIcon: const Icon(Icons.favorite)));
      await tester.pumpAndSettle();
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(find.byIcon(Icons.favorite), findsOneWidget);
      await tester.tap(find.text('OK'));
      await tester.pumpAndSettle();
    });
  });

  group('Calendar mode', () {
    testWidgets('Default Calendar mode layout (Landscape)', (WidgetTester tester) async {
      final Finder helpText = find.text('Select date');
      final Finder headerText = find.text('Fri, Jan 15');
      final Finder subHeaderText = find.text('January 2016');
      final Finder cancelButtonText = find.text('Cancel');
      final Finder okButtonText = find.text('OK');
      const EdgeInsets insetPadding = EdgeInsets.symmetric(horizontal: 16.0, vertical: 24.0);

      tester.view.physicalSize = wideWindowSize;
      addTearDown(tester.view.reset);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: true),
          home: Material(
            child: DatePickerDialog(
              initialDate: initialDate,
              firstDate: firstDate,
              lastDate: lastDate,
            ),
          ),
        ),
      );

      expect(helpText, findsOneWidget);
      expect(headerText, findsOneWidget);
      expect(subHeaderText, findsOneWidget);
      expect(cancelButtonText, findsOneWidget);
      expect(okButtonText, findsOneWidget);

      // Test help text position.
      final Offset dialogTopLeft = tester.getTopLeft(find.byType(AnimatedContainer));
      final Offset helpTextTopLeft = tester.getTopLeft(helpText);
      expect(helpTextTopLeft.dx, dialogTopLeft.dx + (insetPadding.horizontal / 2));
      expect(helpTextTopLeft.dy, dialogTopLeft.dy + 16.0);

      // Test header text position.
      final Offset headerTextTopLeft = tester.getTopLeft(headerText);
      final Offset helpTextBottomLeft = tester.getBottomLeft(helpText);
      expect(headerTextTopLeft.dx, dialogTopLeft.dx + (insetPadding.horizontal / 2));
      expect(headerTextTopLeft.dy, helpTextBottomLeft.dy + 16.0);

      // Test switch button position.
      final Finder switchButtonM3 = find.widgetWithIcon(IconButton, Icons.edit_outlined);
      final Offset switchButtonTopLeft = tester.getTopLeft(switchButtonM3);
      final Offset switchButtonBottomLeft = tester.getBottomLeft(switchButtonM3);
      final Offset headerTextBottomLeft = tester.getBottomLeft(headerText);
      final Offset dialogBottomLeft = tester.getBottomLeft(find.byType(AnimatedContainer));
      expect(switchButtonTopLeft.dx, dialogTopLeft.dx + 8.0);
      expect(switchButtonTopLeft.dy, headerTextBottomLeft.dy);
      expect(switchButtonBottomLeft.dx, dialogTopLeft.dx + 8.0);
      expect(switchButtonBottomLeft.dy, dialogBottomLeft.dy - 6.0);

      // Test vertical divider position.
      final Finder divider = find.byType(VerticalDivider);
      final Offset dividerTopLeft = tester.getTopLeft(divider);
      final Offset headerTextTopRight = tester.getTopRight(headerText);
      expect(dividerTopLeft.dx, headerTextTopRight.dx + 16.0);
      expect(dividerTopLeft.dy, dialogTopLeft.dy);

      // Test sub header text position.
      final Offset subHeaderTextTopLeft = tester.getTopLeft(subHeaderText);
      final Offset dividerTopRight = tester.getTopRight(divider);
      expect(subHeaderTextTopLeft.dx, dividerTopRight.dx + 24.0);
      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(subHeaderTextTopLeft.dy, dialogTopLeft.dy + 16.0);
      }

      // Test sub header icon position.
      final Finder subHeaderIcon = find.byIcon(Icons.arrow_drop_down);
      final Offset subHeaderIconTopLeft = tester.getTopLeft(subHeaderIcon);
      final Offset subHeaderTextTopRight = tester.getTopRight(subHeaderText);
      expect(subHeaderIconTopLeft.dx, subHeaderTextTopRight.dx);
      expect(subHeaderIconTopLeft.dy, dialogTopLeft.dy + 14.0);

      // Test calendar page view position.
      final Finder calendarPageView = find.byType(PageView);
      final Offset calendarPageViewTopLeft = tester.getTopLeft(calendarPageView);
      final Offset subHeaderTextBottomLeft = tester.getBottomLeft(subHeaderText);
      expect(calendarPageViewTopLeft.dx, dividerTopRight.dx);
      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(calendarPageViewTopLeft.dy, subHeaderTextBottomLeft.dy + 16.0);
      }

      // Test month navigation icons position.
      final Finder previousMonthButton = find.widgetWithIcon(IconButton, Icons.chevron_left);
      final Finder nextMonthButton = find.widgetWithIcon(IconButton, Icons.chevron_right);
      final Offset previousMonthButtonTopRight = tester.getTopRight(previousMonthButton);
      final Offset nextMonthButtonTopRight = tester.getTopRight(nextMonthButton);
      final Offset dialogTopRight = tester.getTopRight(find.byType(AnimatedContainer));
      expect(nextMonthButtonTopRight.dx, dialogTopRight.dx - 4.0);
      expect(nextMonthButtonTopRight.dy, dialogTopRight.dy + 2.0);
      expect(previousMonthButtonTopRight.dx, nextMonthButtonTopRight.dx - 48.0);

      // Test action buttons position.
      final Offset dialogBottomRight = tester.getBottomRight(find.byType(AnimatedContainer));
      final Offset okButtonTopRight = tester.getTopRight(find.widgetWithText(TextButton, 'OK'));
      final Offset cancelButtonTopRight = tester.getTopRight(
        find.widgetWithText(TextButton, 'Cancel'),
      );
      final Offset calendarPageViewBottomRight = tester.getBottomRight(calendarPageView);
      expect(okButtonTopRight.dx, dialogBottomRight.dx - 8);
      expect(okButtonTopRight.dy, calendarPageViewBottomRight.dy + 2);
      final Offset okButtonTopLeft = tester.getTopLeft(find.widgetWithText(TextButton, 'OK'));
      expect(cancelButtonTopRight.dx, okButtonTopLeft.dx - 8);
    });

    testWidgets('Default Calendar mode layout (Portrait)', (WidgetTester tester) async {
      final Finder helpText = find.text('Select date');
      final Finder headerText = find.text('Fri, Jan 15');
      final Finder subHeaderText = find.text('January 2016');
      final Finder cancelButtonText = find.text('Cancel');
      final Finder okButtonText = find.text('OK');

      tester.view.physicalSize = narrowWindowSize;
      addTearDown(tester.view.reset);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: true),
          home: Material(
            child: DatePickerDialog(
              initialDate: initialDate,
              firstDate: firstDate,
              lastDate: lastDate,
            ),
          ),
        ),
      );

      expect(helpText, findsOneWidget);
      expect(headerText, findsOneWidget);
      expect(subHeaderText, findsOneWidget);
      expect(cancelButtonText, findsOneWidget);
      expect(okButtonText, findsOneWidget);

      // Test help text position.
      final Offset dialogTopLeft = tester.getTopLeft(find.byType(AnimatedContainer));
      final Offset helpTextTopLeft = tester.getTopLeft(helpText);
      expect(helpTextTopLeft.dx, dialogTopLeft.dx + 24.0);
      expect(helpTextTopLeft.dy, dialogTopLeft.dy + 16.0);

      // Test header text position
      final Offset headerTextTextTopLeft = tester.getTopLeft(headerText);
      final Offset helpTextBottomLeft = tester.getBottomLeft(helpText);
      expect(headerTextTextTopLeft.dx, dialogTopLeft.dx + 24.0);
      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(headerTextTextTopLeft.dy, helpTextBottomLeft.dy + 28.0);
      }

      // Test switch button position.
      final Finder switchButtonM3 = find.widgetWithIcon(IconButton, Icons.edit_outlined);
      final Offset switchButtonTopRight = tester.getTopRight(switchButtonM3);
      final Offset dialogTopRight = tester.getTopRight(find.byType(AnimatedContainer));
      expect(switchButtonTopRight.dx, dialogTopRight.dx - 12.0);
      expect(switchButtonTopRight.dy, headerTextTextTopLeft.dy - 4.0);

      // Test horizontal divider position.
      final Finder divider = find.byType(Divider);
      final Offset dividerTopLeft = tester.getTopLeft(divider);
      final Offset headerTextBottomLeft = tester.getBottomLeft(headerText);
      expect(dividerTopLeft.dx, dialogTopLeft.dx);
      expect(dividerTopLeft.dy, headerTextBottomLeft.dy + 16.0);

      // Test subHeaderText position.
      final Offset subHeaderTextTopLeft = tester.getTopLeft(subHeaderText);
      final Offset dividerBottomLeft = tester.getBottomLeft(divider);
      expect(subHeaderTextTopLeft.dx, dialogTopLeft.dx + 24.0);
      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(subHeaderTextTopLeft.dy, dividerBottomLeft.dy + 16.0);
      }

      // Test sub header icon position.
      final Finder subHeaderIcon = find.byIcon(Icons.arrow_drop_down);
      final Offset subHeaderIconTopLeft = tester.getTopLeft(subHeaderIcon);
      final Offset subHeaderTextTopRight = tester.getTopRight(subHeaderText);
      expect(subHeaderIconTopLeft.dx, subHeaderTextTopRight.dx);
      expect(subHeaderIconTopLeft.dy, dividerBottomLeft.dy + 14.0);

      // Test month navigation icons position.
      final Finder previousMonthButton = find.widgetWithIcon(IconButton, Icons.chevron_left);
      final Finder nextMonthButton = find.widgetWithIcon(IconButton, Icons.chevron_right);
      final Offset previousMonthButtonTopRight = tester.getTopRight(previousMonthButton);
      final Offset nextMonthButtonTopRight = tester.getTopRight(nextMonthButton);
      expect(nextMonthButtonTopRight.dx, dialogTopRight.dx - 4.0);
      expect(nextMonthButtonTopRight.dy, dividerBottomLeft.dy + 2.0);
      expect(previousMonthButtonTopRight.dx, nextMonthButtonTopRight.dx - 48.0);

      // Test calendar page view position.
      final Finder calendarPageView = find.byType(PageView);
      final Offset calendarPageViewTopLeft = tester.getTopLeft(calendarPageView);
      final Offset subHeaderTextBottomLeft = tester.getBottomLeft(subHeaderText);
      expect(calendarPageViewTopLeft.dx, dialogTopLeft.dx);
      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(calendarPageViewTopLeft.dy, subHeaderTextBottomLeft.dy + 16.0);
      }

      // Test action buttons position.
      final Offset dialogBottomRight = tester.getBottomRight(find.byType(AnimatedContainer));
      final Offset okButtonTopRight = tester.getTopRight(find.widgetWithText(TextButton, 'OK'));
      final Offset cancelButtonTopRight = tester.getTopRight(
        find.widgetWithText(TextButton, 'Cancel'),
      );
      final Offset calendarPageViewBottomRight = tester.getBottomRight(calendarPageView);
      final Offset okButtonTopLeft = tester.getTopLeft(find.widgetWithText(TextButton, 'OK'));
      expect(okButtonTopRight.dx, dialogBottomRight.dx - 8);
      expect(okButtonTopRight.dy, calendarPageViewBottomRight.dy + 2);
      expect(cancelButtonTopRight.dx, okButtonTopLeft.dx - 8);
    });

    testWidgets('Can select a day', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('12'));
        await tester.tap(find.text('OK'));
        expect(await date, equals(DateTime(2016, DateTime.january, 12)));
      });
    });

    testWidgets('Can select a month', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(previousMonthIcon);
        await tester.pumpAndSettle(const Duration(seconds: 1));
        await tester.tap(find.text('25'));
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2015, DateTime.december, 25));
      });
    });

    testWidgets('Can select a year', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('January 2016')); // Switch to year mode.
        await tester.pump();
        await tester.tap(find.text('2018'));
        await tester.pump();
        expect(find.text('January 2018'), findsOneWidget);
      });
    });

    testWidgets('Can select a day with no initial date', (WidgetTester tester) async {
      initialDate = null;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('12'));
        await tester.tap(find.text('OK'));
        expect(await date, equals(DateTime(2016, DateTime.january, 12)));
      });
    });

    testWidgets('Can select a month with no initial date', (WidgetTester tester) async {
      initialDate = null;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(previousMonthIcon);
        await tester.pumpAndSettle(const Duration(seconds: 1));
        await tester.tap(find.text('25'));
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2015, DateTime.december, 25));
      });
    });

    testWidgets('Can select a year with no initial date', (WidgetTester tester) async {
      initialDate = null;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('January 2016')); // Switch to year mode.
        await tester.pump();
        await tester.tap(find.text('2018'));
        await tester.pump();
        expect(find.text('January 2018'), findsOneWidget);
      });
    });

    testWidgets('Selecting date does not change displayed month', (WidgetTester tester) async {
      initialDate = DateTime(2020, DateTime.march, 15);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(nextMonthIcon);
        await tester.pumpAndSettle(const Duration(seconds: 1));
        expect(find.text('April 2020'), findsOneWidget);
        await tester.tap(find.text('25'));
        await tester.pumpAndSettle();
        expect(find.text('April 2020'), findsOneWidget);
        // There isn't a 31 in April so there shouldn't be one if it is showing April
        expect(find.text('31'), findsNothing);
      });
    });

    testWidgets('Changing year does change selected date', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('January 2016'));
        await tester.pump();
        await tester.tap(find.text('2018'));
        await tester.pump();
        await tester.tap(find.text('OK'));
        expect(await date, equals(DateTime(2018, DateTime.january, 15)));
      });
    });

    testWidgets('Changing year does not change the month', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(nextMonthIcon);
        await tester.pumpAndSettle();
        await tester.tap(nextMonthIcon);
        await tester.pumpAndSettle();
        await tester.tap(find.text('March 2016'));
        await tester.pumpAndSettle();
        await tester.tap(find.text('2018'));
        await tester.pumpAndSettle();
        expect(find.text('March 2018'), findsOneWidget);
      });
    });

    testWidgets('Can select a year and then a day', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('January 2016')); // Switch to year mode.
        await tester.pump();
        await tester.tap(find.text('2017'));
        await tester.pump();
        await tester.tap(find.text('19'));
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2017, DateTime.january, 19));
      });
    });

    testWidgets('Current year is visible in year picker', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('January 2016')); // Switch to year mode.
        await tester.pump();
        expect(find.text('2016'), findsOneWidget);
      });
    });

    testWidgets('Cannot select a day outside bounds', (WidgetTester tester) async {
      initialDate = DateTime(2017, DateTime.january, 15);
      firstDate = initialDate!;
      lastDate = initialDate!;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Earlier than firstDate. Should be ignored.
        await tester.tap(find.text('10'));
        // Later than lastDate. Should be ignored.
        await tester.tap(find.text('20'));
        await tester.tap(find.text('OK'));
        // We should still be on the initial date.
        expect(await date, initialDate);
      });
    });

    testWidgets('Cannot select a month past last date', (WidgetTester tester) async {
      initialDate = DateTime(2017, DateTime.january, 15);
      firstDate = initialDate!;
      lastDate = DateTime(2017, DateTime.february, 20);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(nextMonthIcon);
        await tester.pumpAndSettle(const Duration(seconds: 1));
        // Shouldn't be possible to keep going into March.
        expect(nextMonthIcon, findsNothing);
      });
    });

    testWidgets('Cannot select a month before first date', (WidgetTester tester) async {
      initialDate = DateTime(2017, DateTime.january, 15);
      firstDate = DateTime(2016, DateTime.december, 10);
      lastDate = initialDate!;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(previousMonthIcon);
        await tester.pumpAndSettle(const Duration(seconds: 1));
        // Shouldn't be possible to keep going into November.
        expect(previousMonthIcon, findsNothing);
      });
    });

    testWidgets('Cannot select disabled year', (WidgetTester tester) async {
      initialDate = DateTime(2018, DateTime.july, 4);
      firstDate = DateTime(2018, DateTime.june, 9);
      lastDate = DateTime(2018, DateTime.december, 15);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('July 2018')); // Switch to year mode.
        await tester.pumpAndSettle();
        await tester.tap(find.text('2016')); // Disabled, doesn't change the year.
        await tester.pumpAndSettle();
        await tester.tap(find.text('OK'));
        await tester.pumpAndSettle();
        expect(await date, DateTime(2018, DateTime.july, 4));
      });
    });

    testWidgets('Selecting firstDate year respects firstDate', (WidgetTester tester) async {
      initialDate = DateTime(2018, DateTime.may, 4);
      firstDate = DateTime(2016, DateTime.june, 9);
      lastDate = DateTime(2019, DateTime.january, 15);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('May 2018'));
        await tester.pumpAndSettle();
        await tester.tap(find.text('2016'));
        await tester.pumpAndSettle();
        // Month should be clamped to June as the range starts at June 2016
        expect(find.text('June 2016'), findsOneWidget);
      });
    });

    testWidgets('Selecting lastDate year respects lastDate', (WidgetTester tester) async {
      initialDate = DateTime(2018, DateTime.may, 4);
      firstDate = DateTime(2016, DateTime.june, 9);
      lastDate = DateTime(2019, DateTime.january, 15);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('May 2018'));
        await tester.pumpAndSettle();
        await tester.tap(find.text('2019'));
        await tester.pumpAndSettle();
        // Month should be clamped to January as the range ends at January 2019
        expect(find.text('January 2019'), findsOneWidget);
      });
    });

    testWidgets('Only predicate days are selectable', (WidgetTester tester) async {
      initialDate = DateTime(2017, DateTime.january, 16);
      firstDate = DateTime(2017, DateTime.january, 10);
      lastDate = DateTime(2017, DateTime.january, 20);
      selectableDayPredicate = (DateTime day) => day.day.isEven;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('13')); // Odd, doesn't work.
        await tester.tap(find.text('10')); // Even, works.
        await tester.tap(find.text('17')); // Odd, doesn't work.
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2017, DateTime.january, 10));
      });
    });

    testWidgets('Can select initial calendar picker mode', (WidgetTester tester) async {
      initialDate = DateTime(2014, DateTime.january, 15);
      initialCalendarMode = DatePickerMode.year;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.pump();
        // 2018 wouldn't be available if the year picker wasn't showing.
        // The initial current year is 2014.
        await tester.tap(find.text('2018'));
        await tester.pump();
        expect(find.text('January 2018'), findsOneWidget);
      });
    });

    testWidgets('currentDate is highlighted', (WidgetTester tester) async {
      today = DateTime(2016, 1, 2);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.pump();
        const Color todayColor = Color(0xff2196f3); // default primary color
        expect(
          findDayGridMaterial(tester),
          // The current day should be painted with a circle outline
          paints..circle(color: todayColor, style: PaintingStyle.stroke, strokeWidth: 1.0),
        );
      });
    });

    testWidgets('Date picker dayOverlayColor resolves hovered state', (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      final Offset center = tester.getCenter(find.text('30'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.moveTo(center);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.08)),
      );
    });

    testWidgets('Date picker dayOverlayColor resolves focused state', (WidgetTester tester) async {
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      final ThemeData theme = ThemeData();
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      // Navigate to the grid.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);

      // Navigate to day 30.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.10)),
      );
    });

    testWidgets('Date picker dayOverlayColor resolves pressed state', (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      final Offset center = tester.getCenter(find.text('30'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.down(center);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle() // Hovered decoration.
          ..circle(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.10)),
      );
      await gesture.up();
    });

    // Regression test for https://github.com/flutter/flutter/issues/130586.
    testWidgets('Date picker dayOverlayColor resolves selected and hovered state', (
      WidgetTester tester,
    ) async {
      final ThemeData theme = ThemeData();
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      // Select day 30.
      await tester.tap(find.text('30'));
      await tester.pumpAndSettle();
      final ShapeDecoration day30Decoration = findDayDecoration(tester, '30')!;
      expect(day30Decoration.color, theme.colorScheme.primary);

      final Offset center = tester.getCenter(find.text('30'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.moveTo(center);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: theme.colorScheme.onPrimary.withOpacity(0.08)),
      );
    });

    testWidgets('Date picker dayOverlayColor resolves selected and focused state', (
      WidgetTester tester,
    ) async {
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      final ThemeData theme = ThemeData();
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      // Select day 30.
      await tester.tap(find.text('30'));
      await tester.pumpAndSettle();
      final ShapeDecoration day30Decoration = findDayDecoration(tester, '30')!;
      expect(day30Decoration.color, theme.colorScheme.primary);

      // Navigate to the grid.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();

      // Day 30 is selected and focused.
      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: theme.colorScheme.onPrimary.withOpacity(0.10)),
      );
    });

    testWidgets('Date picker dayOverlayColor resolves selected and pressed state', (
      WidgetTester tester,
    ) async {
      final ThemeData theme = ThemeData();
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      // Select day 30.
      await tester.tap(find.text('30'));
      await tester.pumpAndSettle();
      final ShapeDecoration day30Decoration = findDayDecoration(tester, '30')!;
      expect(day30Decoration.color, theme.colorScheme.primary);

      final Offset center = tester.getCenter(find.text('30'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.down(center);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle() // Hovered decoration.
          ..circle(color: theme.colorScheme.onPrimary.withOpacity(0.10)),
      );
      await gesture.up();
    });

    testWidgets('Selecting date does not switch picker to year selection', (
      WidgetTester tester,
    ) async {
      initialDate = DateTime(2020, DateTime.may, 10);
      initialCalendarMode = DatePickerMode.year;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.pump();
        await tester.tap(find.text('2017'));
        await tester.pump();
        expect(find.text('May 2017'), findsOneWidget);
        await tester.tap(find.text('10'));
        await tester.pump();
        expect(find.text('May 2017'), findsOneWidget);
        expect(find.text('2017'), findsNothing);
      });
    });

    testWidgets('Calendar dialog contents are visible - textScaler 0.88, 1.0, 2.0', (
      WidgetTester tester,
    ) async {
      addTearDown(tester.view.reset);
      tester.view.physicalSize = const Size(400, 800);
      tester.view.devicePixelRatio = 1.0;
      final List<double> scales = <double>[0.88, 1.0, 2.0];

      for (final double scale in scales) {
        await tester.pumpWidget(
          MaterialApp(
            home: MediaQuery(
              data: MediaQueryData(textScaler: TextScaler.linear(scale)),
              child: Material(
                child: DatePickerDialog(
                  firstDate: DateTime(2001),
                  lastDate: DateTime(2031, DateTime.december, 31),
                  initialDate: DateTime(2016, DateTime.january, 15),
                  initialEntryMode: DatePickerEntryMode.calendarOnly,
                ),
              ),
            ),
          ),
        );
        await tester.pumpAndSettle();

        await expectLater(
          find.byType(Dialog),
          matchesGoldenFile('date_picker.calendar.contents.visible.$scale.png'),
        );
      }
    });
  });

  group('Input mode', () {
    setUp(() {
      firstDate = DateTime(2015);
      lastDate = DateTime(2017, DateTime.december, 31);
      initialDate = DateTime(2016, DateTime.january, 15);
      initialEntryMode = DatePickerEntryMode.input;
    });

    testWidgets('Default InputDecoration', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final InputDecoration decoration =
            tester.widget<TextField>(find.byType(TextField)).decoration!;
        expect(decoration.border, const OutlineInputBorder());
        expect(decoration.filled, false);
        expect(decoration.hintText, 'mm/dd/yyyy');
        expect(decoration.labelText, 'Enter Date');
        expect(decoration.errorText, null);
      }, useMaterial3: true);
    });

    testWidgets('Initial entry mode is used', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.byType(TextField), findsOneWidget);
      });
    });

    testWidgets('Hint, label, and help text is used', (WidgetTester tester) async {
      cancelText = 'nope';
      confirmText = 'yep';
      fieldHintText = 'hint';
      fieldLabelText = 'label';
      helpText = 'help';
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.text(cancelText!), findsOneWidget);
        expect(find.text(confirmText!), findsOneWidget);
        expect(find.text(fieldHintText!), findsOneWidget);
        expect(find.text(fieldLabelText!), findsOneWidget);
        expect(find.text(helpText!), findsOneWidget);
      });
    });

    testWidgets('KeyboardType is used', (WidgetTester tester) async {
      keyboardType = TextInputType.text;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final TextField field = textField(tester);
        expect(field.keyboardType, TextInputType.text);
      });
    });

    testWidgets('Initial date is the default', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2016, DateTime.january, 15));
      });
    });

    testWidgets('Can toggle to calendar entry mode', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.byType(TextField), findsOneWidget);
        await tester.tap(find.byIcon(Icons.calendar_today));
        await tester.pumpAndSettle();
        expect(find.byType(TextField), findsNothing);
      });
    });

    testWidgets('Toggle to calendar mode keeps selected date', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final TextField field = textField(tester);
        field.controller!.clear();

        await tester.enterText(find.byType(TextField), '12/25/2016');
        await tester.tap(find.byIcon(Icons.calendar_today));
        await tester.pumpAndSettle();
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2016, DateTime.december, 25));
      });
    });

    testWidgets('Entered text returns date', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final TextField field = textField(tester);
        field.controller!.clear();

        await tester.enterText(find.byType(TextField), '12/25/2016');
        await tester.tap(find.text('OK'));
        expect(await date, DateTime(2016, DateTime.december, 25));
      });
    });

    testWidgets('Too short entered text shows error', (WidgetTester tester) async {
      errorFormatText = 'oops';
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final TextField field = textField(tester);
        field.controller!.clear();

        await tester.pumpAndSettle();
        await tester.enterText(find.byType(TextField), '1225');
        expect(find.text(errorFormatText!), findsNothing);

        await tester.tap(find.text('OK'));
        await tester.pumpAndSettle();
        expect(find.text(errorFormatText!), findsOneWidget);
      });
    });

    testWidgets('Bad format entered text shows error', (WidgetTester tester) async {
      errorFormatText = 'oops';
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final TextField field = textField(tester);
        field.controller!.clear();

        await tester.pumpAndSettle();
        await tester.enterText(find.byType(TextField), '20 days, 3 months, 2003');
        expect(find.text('20 days, 3 months, 2003'), findsOneWidget);
        expect(find.text(errorFormatText!), findsNothing);

        await tester.tap(find.text('OK'));
        await tester.pumpAndSettle();
        expect(find.text(errorFormatText!), findsOneWidget);
      });
    });

    testWidgets('Invalid entered text shows error', (WidgetTester tester) async {
      errorInvalidText = 'oops';
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final TextField field = textField(tester);
        field.controller!.clear();

        await tester.pumpAndSettle();
        await tester.enterText(find.byType(TextField), '08/10/1969');
        expect(find.text(errorInvalidText!), findsNothing);

        await tester.tap(find.text('OK'));
        await tester.pumpAndSettle();
        expect(find.text(errorInvalidText!), findsOneWidget);
      });
    });

    testWidgets('Invalid entered text shows error on autovalidate', (WidgetTester tester) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/126397.
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        final TextField field = textField(tester);
        field.controller!.clear();

        // Enter some text to trigger autovalidate.
        await tester.enterText(find.byType(TextField), 'xyz');
        await tester.tap(find.text('OK'));
        await tester.pumpAndSettle();

        // Invalid format validation error should be shown.
        expect(find.text('Invalid format.'), findsOneWidget);

        // Clear the text.
        field.controller!.clear();

        // Enter an invalid date that is too long while autovalidate is still on.
        await tester.enterText(find.byType(TextField), '10/05/2023666777889');
        await tester.pump();

        // Invalid format validation error should be shown.
        expect(find.text('Invalid format.'), findsOneWidget);
        // Should not throw an exception.
        expect(tester.takeException(), null);
      });
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/131989.
    testWidgets('Dialog contents do not overflow when resized during orientation change', (
      WidgetTester tester,
    ) async {
      addTearDown(tester.view.reset);
      // Initial window size is wide for landscape mode.
      tester.view.physicalSize = wideWindowSize;
      tester.view.devicePixelRatio = 1.0;

      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Change window size to narrow for portrait mode.
        tester.view.physicalSize = narrowWindowSize;
        await tester.pump();
        expect(tester.takeException(), null);
      });
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/139120.
    testWidgets('Dialog contents are visible - textScaler 0.88, 1.0, 2.0', (
      WidgetTester tester,
    ) async {
      addTearDown(tester.view.reset);
      tester.view.physicalSize = const Size(400, 800);
      tester.view.devicePixelRatio = 1.0;
      final List<double> scales = <double>[0.88, 1.0, 2.0];

      for (final double scale in scales) {
        await tester.pumpWidget(
          MaterialApp(
            home: MediaQuery(
              data: MediaQueryData(textScaler: TextScaler.linear(scale)),
              child: Material(
                child: DatePickerDialog(
                  firstDate: DateTime(2001),
                  lastDate: DateTime(2031, DateTime.december, 31),
                  initialDate: DateTime(2016, DateTime.january, 15),
                  initialEntryMode: DatePickerEntryMode.input,
                ),
              ),
            ),
          ),
        );
        await tester.pumpAndSettle();

        await expectLater(
          find.byType(Dialog),
          matchesGoldenFile('date_picker.dialog.contents.visible.$scale.png'),
        );
      }
    });
  });

  group('Semantics', () {
    testWidgets('calendar mode', (WidgetTester tester) async {
      final SemanticsHandle semantics = tester.ensureSemantics();

      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Header
        expect(
          tester.getSemantics(find.text('SELECT DATE')),
          matchesSemantics(label: 'SELECT DATE\nFri, Jan 15'),
        );

        expect(
          tester.getSemantics(find.text('3')),
          matchesSemantics(
            label: '3, Sunday, January 3, 2016, Today',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );

        // Input mode toggle button
        expect(
          tester.getSemantics(switchToInputIcon),
          matchesSemantics(
            tooltip: 'Switch to input',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isEnabled: true,
            hasEnabledState: true,
            isFocusable: true,
          ),
        );

        // The semantics of the CalendarDatePicker are tested in its tests.

        // Ok/Cancel buttons
        expect(
          tester.getSemantics(find.text('OK')),
          matchesSemantics(
            label: 'OK',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isEnabled: true,
            hasEnabledState: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('CANCEL')),
          matchesSemantics(
            label: 'CANCEL',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isEnabled: true,
            hasEnabledState: true,
            isFocusable: true,
          ),
        );
      });
      semantics.dispose();
    });

    testWidgets('input mode', (WidgetTester tester) async {
      // Fill the clipboard so that the Paste option is available in the text
      // selection menu.
      final MockClipboard mockClipboard = MockClipboard();
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        mockClipboard.handleMethodCall,
      );
      await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.platform,
          null,
        ),
      );

      final SemanticsHandle semantics = tester.ensureSemantics();

      initialEntryMode = DatePickerEntryMode.input;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Header
        expect(
          tester.getSemantics(find.text('SELECT DATE')),
          matchesSemantics(label: 'SELECT DATE\nFri, Jan 15'),
        );

        // Input mode toggle button
        expect(
          tester.getSemantics(switchToCalendarIcon),
          matchesSemantics(
            tooltip: 'Switch to calendar',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isEnabled: true,
            hasEnabledState: true,
            isFocusable: true,
          ),
        );

        expect(
          tester.getSemantics(find.byType(EditableText)),
          matchesSemantics(
            label: 'Enter Date',
            isEnabled: true,
            hasEnabledState: true,
            isTextField: true,
            isFocused: true,
            value: '01/15/2016',
            hasTapAction: true,
            hasFocusAction: true,
            hasSetTextAction: true,
            hasSetSelectionAction: true,
            hasCopyAction: true,
            hasCutAction: true,
            hasPasteAction: true,
            hasMoveCursorBackwardByCharacterAction: true,
            hasMoveCursorBackwardByWordAction: true,
          ),
        );

        // Ok/Cancel buttons
        expect(
          tester.getSemantics(find.text('OK')),
          matchesSemantics(
            label: 'OK',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isEnabled: true,
            hasEnabledState: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('CANCEL')),
          matchesSemantics(
            label: 'CANCEL',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isEnabled: true,
            hasEnabledState: true,
            isFocusable: true,
          ),
        );
      });
      semantics.dispose();
    });

    // Regression test for https://github.com/flutter/flutter/pull/152705
    testWidgets('datepicker dialog semantics node not focusable', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: DatePickerDialog(
              initialDate: initialDate,
              firstDate: firstDate,
              lastDate: lastDate,
            ),
          ),
        ),
      );

      final SemanticsNode node = tester.semantics.find(find.byType(DatePickerDialog));
      final SemanticsData semanticsData = node.getSemanticsData();
      expect(semanticsData.hasFlag(SemanticsFlag.isFocusable), false);
    });
  });

  group('Keyboard navigation', () {
    testWidgets('Can toggle to calendar entry mode', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.byType(TextField), findsNothing);
        // Navigate to the entry toggle button and activate it
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        // Should be in the input mode
        expect(find.byType(TextField), findsOneWidget);
      });
    });

    testWidgets('Can toggle to year mode', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.text('2016'), findsNothing);
        // Navigate to the year selector and activate it
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        // The years should be visible
        expect(find.text('2016'), findsOneWidget);
      });
    });

    testWidgets('Can navigate next/previous months', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        expect(find.text('January 2016'), findsOneWidget);
        // Navigate to the previous month button and activate it twice
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        // Should be showing Nov 2015
        expect(find.text('November 2015'), findsOneWidget);

        // Navigate to the next month button and activate it four times
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        // Should be on Mar 2016
        expect(find.text('March 2016'), findsOneWidget);
      });
    });

    testWidgets('Can navigate date grid with arrow keys', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Navigate to the grid
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);

        // Navigate from Jan 15 to Jan 18 with arrow keys
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();

        // Activate it
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Navigate out of the grid and to the OK button
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);

        // Activate OK
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Should have selected Jan 18
        expect(await date, DateTime(2016, DateTime.january, 18));
      });
    });

    testWidgets('Navigating with arrow keys scrolls months', (WidgetTester tester) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Navigate to the grid
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pumpAndSettle();

        // Navigate from Jan 15 to Dec 31 with arrow keys
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();

        // Should have scrolled to Dec 2015
        expect(find.text('December 2015'), findsOneWidget);

        // Navigate from Dec 31 to Nov 26 with arrow keys
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.pumpAndSettle();

        // Should have scrolled to Nov 2015
        expect(find.text('November 2015'), findsOneWidget);

        // Activate it
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Navigate out of the grid and to the OK button
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pumpAndSettle();

        // Activate OK
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Should have selected Jan 18
        expect(await date, DateTime(2015, DateTime.november, 26));
      });
    });

    testWidgets('RTL text direction reverses the horizontal arrow key navigation', (
      WidgetTester tester,
    ) async {
      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        // Navigate to the grid
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pumpAndSettle();

        // Navigate from Jan 15 to 19 with arrow keys
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();

        // Activate it
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Navigate out of the grid and to the OK button
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pumpAndSettle();

        // Activate OK
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Should have selected Jan 18
        expect(await date, DateTime(2016, DateTime.january, 19));
      }, textDirection: TextDirection.rtl);
    });
  });

  group('Screen configurations', () {
    // Test various combinations of screen sizes, orientations and text scales
    // to ensure the layout doesn't overflow and cause an exception to be thrown.

    // Regression tests for https://github.com/flutter/flutter/issues/21383
    // Regression tests for https://github.com/flutter/flutter/issues/19744
    // Regression tests for https://github.com/flutter/flutter/issues/17745

    // Common screen size roughly based on a Pixel 1
    const Size kCommonScreenSizePortrait = Size(1070, 1770);
    const Size kCommonScreenSizeLandscape = Size(1770, 1070);

    // Small screen size based on a LG K130
    const Size kSmallScreenSizePortrait = Size(320, 521);
    const Size kSmallScreenSizeLandscape = Size(521, 320);

    Future<void> showPicker(WidgetTester tester, Size size, [double textScaleFactor = 1.0]) async {
      tester.view.physicalSize = size;
      tester.view.devicePixelRatio = 1.0;
      addTearDown(tester.view.reset);

      await prepareDatePicker(tester, (Future<DateTime?> date) async {
        await tester.tap(find.text('OK'));
      });
      await tester.pumpAndSettle();
    }

    testWidgets('common screen size - portrait', (WidgetTester tester) async {
      await showPicker(tester, kCommonScreenSizePortrait);
      expect(tester.takeException(), isNull);
    });

    testWidgets('common screen size - landscape', (WidgetTester tester) async {
      await showPicker(tester, kCommonScreenSizeLandscape);
      expect(tester.takeException(), isNull);
    });

    testWidgets('common screen size - portrait - textScale 1.3', (WidgetTester tester) async {
      await showPicker(tester, kCommonScreenSizePortrait, 1.3);
      expect(tester.takeException(), isNull);
    });

    testWidgets('common screen size - landscape - textScale 1.3', (WidgetTester tester) async {
      await showPicker(tester, kCommonScreenSizeLandscape, 1.3);
      expect(tester.takeException(), isNull);
    });

    testWidgets('small screen size - portrait', (WidgetTester tester) async {
      await showPicker(tester, kSmallScreenSizePortrait);
      expect(tester.takeException(), isNull);
    });

    testWidgets('small screen size - landscape', (WidgetTester tester) async {
      await showPicker(tester, kSmallScreenSizeLandscape);
      expect(tester.takeException(), isNull);
    });

    testWidgets('small screen size - portrait -textScale 1.3', (WidgetTester tester) async {
      await showPicker(tester, kSmallScreenSizePortrait, 1.3);
      expect(tester.takeException(), isNull);
    });

    testWidgets('small screen size - landscape - textScale 1.3', (WidgetTester tester) async {
      await showPicker(tester, kSmallScreenSizeLandscape, 1.3);
      expect(tester.takeException(), isNull);
    });
  });

  group('showDatePicker avoids overlapping display features', () {
    testWidgets('positioning with anchorPoint', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showDatePicker(
        context: context,
        initialDate: DateTime.now(),
        firstDate: DateTime(2018),
        lastDate: DateTime(2030),
        anchorPoint: const Offset(1000, 0),
      );
      await tester.pumpAndSettle();

      // Should take the right side of the screen
      expect(tester.getTopLeft(find.byType(DatePickerDialog)), const Offset(410.0, 0.0));
      expect(tester.getBottomRight(find.byType(DatePickerDialog)), const Offset(800.0, 600.0));
    });

    testWidgets('positioning with Directionality', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: Directionality(textDirection: TextDirection.rtl, child: child!),
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showDatePicker(
        context: context,
        initialDate: DateTime.now(),
        firstDate: DateTime(2018),
        lastDate: DateTime(2030),
      );
      await tester.pumpAndSettle();

      // By default it should place the dialog on the right screen
      expect(tester.getTopLeft(find.byType(DatePickerDialog)), const Offset(410.0, 0.0));
      expect(tester.getBottomRight(find.byType(DatePickerDialog)), const Offset(800.0, 600.0));
    });

    testWidgets('positioning with defaults', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showDatePicker(
        context: context,
        initialDate: DateTime.now(),
        firstDate: DateTime(2018),
        lastDate: DateTime(2030),
      );
      await tester.pumpAndSettle();

      // By default it should place the dialog on the left screen
      expect(tester.getTopLeft(find.byType(DatePickerDialog)), Offset.zero);
      expect(tester.getBottomRight(find.byType(DatePickerDialog)), const Offset(390.0, 600.0));
    });
  });

  testWidgets('DatePickerDialog is state restorable', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(restorationScopeId: 'app', home: _RestorableDatePickerDialogTestWidget()),
    );

    // The date picker should be closed.
    expect(find.byType(DatePickerDialog), findsNothing);
    expect(find.text('25/7/2021'), findsOneWidget);

    // Open the date picker.
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();
    expect(find.byType(DatePickerDialog), findsOneWidget);

    final TestRestorationData restorationData = await tester.getRestorationData();
    await tester.restartAndRestore();

    // The date picker should be open after restoring.
    expect(find.byType(DatePickerDialog), findsOneWidget);

    // Tap on the barrier.
    await tester.tapAt(const Offset(10.0, 10.0));
    await tester.pumpAndSettle();

    // The date picker should be closed, the text value updated to the
    // newly selected date.
    expect(find.byType(DatePickerDialog), findsNothing);
    expect(find.text('25/7/2021'), findsOneWidget);

    // The date picker should be open after restoring.
    await tester.restoreFrom(restorationData);
    expect(find.byType(DatePickerDialog), findsOneWidget);

    // Select a different date.
    await tester.tap(find.text('30'));
    await tester.pumpAndSettle();

    // Restart after the new selection. It should remain selected.
    await tester.restartAndRestore();

    // Close the date picker.
    await tester.tap(find.text('OK'));
    await tester.pumpAndSettle();

    // The date picker should be closed, the text value updated to the
    // newly selected date.
    expect(find.byType(DatePickerDialog), findsNothing);
    expect(find.text('30/7/2021'), findsOneWidget);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets('DatePickerDialog state restoration - DatePickerEntryMode', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(
        restorationScopeId: 'app',
        home: _RestorableDatePickerDialogTestWidget(
          datePickerEntryMode: DatePickerEntryMode.calendarOnly,
        ),
      ),
    );

    // The date picker should be closed.
    expect(find.byType(DatePickerDialog), findsNothing);
    expect(find.text('25/7/2021'), findsOneWidget);

    // Open the date picker.
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();
    expect(find.byType(DatePickerDialog), findsOneWidget);

    // Only in calendar mode and cannot switch out.
    expect(find.byType(TextField), findsNothing);
    expect(find.byIcon(Icons.edit), findsNothing);

    final TestRestorationData restorationData = await tester.getRestorationData();
    await tester.restartAndRestore();

    // The date picker should be open after restoring.
    expect(find.byType(DatePickerDialog), findsOneWidget);
    // Only in calendar mode and cannot switch out.
    expect(find.byType(TextField), findsNothing);
    expect(find.byIcon(Icons.edit), findsNothing);

    // Tap on the barrier.
    await tester.tapAt(const Offset(10.0, 10.0));
    await tester.pumpAndSettle();

    // The date picker should be closed, the text value should be the same
    // as before.
    expect(find.byType(DatePickerDialog), findsNothing);
    expect(find.text('25/7/2021'), findsOneWidget);

    // The date picker should be open after restoring.
    await tester.restoreFrom(restorationData);
    expect(find.byType(DatePickerDialog), findsOneWidget);
    // Only in calendar mode and cannot switch out.
    expect(find.byType(TextField), findsNothing);
    expect(find.byIcon(Icons.edit), findsNothing);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets('Test Callback on Toggle of DatePicker Mode', (WidgetTester tester) async {
    prepareDatePicker(tester, (Future<DateTime?> date) async {
      await tester.tap(find.byIcon(Icons.edit));
      expect(currentMode, DatePickerEntryMode.input);
      await tester.pumpAndSettle();
      expect(find.byType(TextField), findsOneWidget);
      await tester.tap(find.byIcon(Icons.calendar_today));
      expect(currentMode, DatePickerEntryMode.calendar);
      await tester.pumpAndSettle();
      expect(find.byType(TextField), findsNothing);
    });
  });

  testWidgets('DatePickerDialog with updated insetPadding', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DatePickerDialog(
            initialDate: initialDate,
            firstDate: firstDate,
            lastDate: lastDate,
            insetPadding: const EdgeInsets.fromLTRB(10.0, 20.0, 30.0, 40.0),
          ),
        ),
      ),
    );

    final Dialog dialog = tester.widget<Dialog>(find.byType(Dialog));
    expect(dialog.insetPadding, const EdgeInsets.fromLTRB(10.0, 20.0, 30.0, 40.0));
  });

  group('Landscape input-only date picker headers use headlineSmall', () {
    // Regression test for https://github.com/flutter/flutter/issues/122056

    // Common screen size roughly based on a Pixel 1
    const Size kCommonScreenSizePortrait = Size(1070, 1770);
    const Size kCommonScreenSizeLandscape = Size(1770, 1070);

    Future<void> showPicker(WidgetTester tester, Size size) async {
      addTearDown(tester.view.reset);
      tester.view.physicalSize = size;
      tester.view.devicePixelRatio = 1.0;
      initialEntryMode = DatePickerEntryMode.input;
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, useMaterial3: true);
    }

    testWidgets('portrait', (WidgetTester tester) async {
      await showPicker(tester, kCommonScreenSizePortrait);
      expect(tester.widget<Text>(find.text('Fri, Jan 15')).style?.fontSize, 32);
      await tester.tap(find.text('Cancel'));
      await tester.pumpAndSettle();
    });

    testWidgets('landscape', (WidgetTester tester) async {
      await showPicker(tester, kCommonScreenSizeLandscape);
      expect(tester.widget<Text>(find.text('Fri, Jan 15')).style?.fontSize, 24);
      await tester.tap(find.text('Cancel'));
      await tester.pumpAndSettle();
    });
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    group('showDatePicker Dialog', () {
      testWidgets('Default dialog size', (WidgetTester tester) async {
        Future<void> showPicker(WidgetTester tester, Size size) async {
          tester.view.physicalSize = size;
          tester.view.devicePixelRatio = 1.0;
          addTearDown(tester.view.reset);
          await prepareDatePicker(tester, (Future<DateTime?> date) async {});
        }

        const Size wideWindowSize = Size(1920.0, 1080.0);
        const Size narrowWindowSize = Size(1070.0, 1770.0);
        const Size calendarLandscapeDialogSize = Size(496.0, 346.0);
        const Size calendarPortraitDialogSizeM2 = Size(330.0, 518.0);

        // Test landscape layout.
        await showPicker(tester, wideWindowSize);

        Size dialogContainerSize = tester.getSize(find.byType(AnimatedContainer));
        expect(dialogContainerSize, calendarLandscapeDialogSize);

        // Close the dialog.
        await tester.tap(find.text('OK'));
        await tester.pumpAndSettle();

        // Test portrait layout.
        await showPicker(tester, narrowWindowSize);

        dialogContainerSize = tester.getSize(find.byType(AnimatedContainer));
        expect(dialogContainerSize, calendarPortraitDialogSizeM2);
      });

      testWidgets('Default dialog properties', (WidgetTester tester) async {
        final ThemeData theme = ThemeData(useMaterial3: false);
        await prepareDatePicker(tester, (Future<DateTime?> date) async {
          final Material dialogMaterial = tester.widget<Material>(
            find.descendant(of: find.byType(Dialog), matching: find.byType(Material)).first,
          );

          expect(dialogMaterial.color, theme.colorScheme.surface);
          expect(dialogMaterial.shadowColor, theme.shadowColor);
          expect(dialogMaterial.elevation, 24.0);
          expect(
            dialogMaterial.shape,
            const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
          );
          expect(dialogMaterial.clipBehavior, Clip.antiAlias);

          final Dialog dialog = tester.widget<Dialog>(find.byType(Dialog));
          expect(dialog.insetPadding, const EdgeInsets.symmetric(horizontal: 16.0, vertical: 24.0));
        }, useMaterial3: theme.useMaterial3);
      });
    });

    group('Input mode', () {
      setUp(() {
        firstDate = DateTime(2015);
        lastDate = DateTime(2017, DateTime.december, 31);
        initialDate = DateTime(2016, DateTime.january, 15);
        initialEntryMode = DatePickerEntryMode.input;
      });

      testWidgets('Default InputDecoration', (WidgetTester tester) async {
        await prepareDatePicker(tester, (Future<DateTime?> date) async {
          final InputDecoration decoration =
              tester.widget<TextField>(find.byType(TextField)).decoration!;
          expect(decoration.border, const UnderlineInputBorder());
          expect(decoration.filled, false);
          expect(decoration.hintText, 'mm/dd/yyyy');
          expect(decoration.labelText, 'Enter Date');
          expect(decoration.errorText, null);
        });
      });
    });

    testWidgets('Date picker dayOverlayColor resolves hovered state', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: false);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      final Offset center = tester.getCenter(find.text('30'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(center);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.08)),
      );
    });

    testWidgets('Date picker dayOverlayColor resolves focused state', (WidgetTester tester) async {
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      final ThemeData theme = ThemeData(useMaterial3: false);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      // Navigate to the grid.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);

      // Navigate to day 30.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.12)),
      );
    });

    testWidgets('Date picker dayOverlayColor resolves pressed state', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: false);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      final Offset center = tester.getCenter(find.text('30'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.down(center);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle() // Hovered decoration.
          ..circle(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.12)),
      );
      await gesture.up();
    });

    // Regression test for https://github.com/flutter/flutter/issues/130586.
    testWidgets('Date picker dayOverlayColor resolves selected and hovered state', (
      WidgetTester tester,
    ) async {
      final ThemeData theme = ThemeData(useMaterial3: false);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      // Select day 30.
      await tester.tap(find.text('30'));
      await tester.pumpAndSettle();
      final ShapeDecoration day30Decoration = findDayDecoration(tester, '30')!;
      expect(day30Decoration.color, theme.colorScheme.primary);

      final Offset center = tester.getCenter(find.text('30'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.moveTo(center);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: theme.colorScheme.onPrimary.withOpacity(0.08)),
      );
    });

    testWidgets('Date picker dayOverlayColor resolves selected and focused state', (
      WidgetTester tester,
    ) async {
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      final ThemeData theme = ThemeData(useMaterial3: false);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      // Select day 30.
      await tester.tap(find.text('30'));
      await tester.pumpAndSettle();
      final ShapeDecoration day30Decoration = findDayDecoration(tester, '30')!;
      expect(day30Decoration.color, theme.colorScheme.primary);

      // Navigate to the grid.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();

      // Day 30 is selected and focused.
      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: theme.colorScheme.onPrimary.withOpacity(0.12)),
      );
    });

    testWidgets('Date picker dayOverlayColor resolves selected and pressed state', (
      WidgetTester tester,
    ) async {
      final ThemeData theme = ThemeData(useMaterial3: false);
      await prepareDatePicker(tester, (Future<DateTime?> date) async {}, theme: theme);

      // Select day 30.
      await tester.tap(find.text('30'));
      await tester.pumpAndSettle();
      final ShapeDecoration day30Decoration = findDayDecoration(tester, '30')!;
      expect(day30Decoration.color, theme.colorScheme.primary);

      final Offset center = tester.getCenter(find.text('30'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.down(center);
      await tester.pumpAndSettle();

      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle() // Hovered decoration.
          ..circle(color: theme.colorScheme.onPrimary.withOpacity(0.38)),
      );
      await gesture.up();
    });
  });
}

class _RestorableDatePickerDialogTestWidget extends StatefulWidget {
  const _RestorableDatePickerDialogTestWidget({
    this.datePickerEntryMode = DatePickerEntryMode.calendar,
  });

  final DatePickerEntryMode datePickerEntryMode;

  @override
  _RestorableDatePickerDialogTestWidgetState createState() =>
      _RestorableDatePickerDialogTestWidgetState();
}

@pragma('vm:entry-point')
class _RestorableDatePickerDialogTestWidgetState
    extends State<_RestorableDatePickerDialogTestWidget>
    with RestorationMixin {
  @override
  String? get restorationId => 'scaffold_state';

  final RestorableDateTime _selectedDate = RestorableDateTime(DateTime(2021, 7, 25));
  late final RestorableRouteFuture<DateTime?> _restorableDatePickerRouteFuture =
      RestorableRouteFuture<DateTime?>(
        onComplete: _selectDate,
        onPresent: (NavigatorState navigator, Object? arguments) {
          return navigator.restorablePush(
            _datePickerRoute,
            arguments: <String, dynamic>{
              'selectedDate': _selectedDate.value.millisecondsSinceEpoch,
              'datePickerEntryMode': widget.datePickerEntryMode.index,
            },
          );
        },
      );

  @override
  void dispose() {
    _selectedDate.dispose();
    _restorableDatePickerRouteFuture.dispose();
    super.dispose();
  }

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_selectedDate, 'selected_date');
    registerForRestoration(_restorableDatePickerRouteFuture, 'date_picker_route_future');
  }

  void _selectDate(DateTime? newSelectedDate) {
    if (newSelectedDate != null) {
      setState(() {
        _selectedDate.value = newSelectedDate;
      });
    }
  }

  @pragma('vm:entry-point')
  static Route<DateTime> _datePickerRoute(BuildContext context, Object? arguments) {
    return DialogRoute<DateTime>(
      context: context,
      builder: (BuildContext context) {
        final Map<dynamic, dynamic> args = arguments! as Map<dynamic, dynamic>;
        return DatePickerDialog(
          restorationId: 'date_picker_dialog',
          initialEntryMode: DatePickerEntryMode.values[args['datePickerEntryMode'] as int],
          initialDate: DateTime.fromMillisecondsSinceEpoch(args['selectedDate'] as int),
          firstDate: DateTime(2021),
          lastDate: DateTime(2022),
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    final DateTime selectedDateTime = _selectedDate.value;
    // Example: "25/7/1994"
    final String selectedDateTimeString =
        '${selectedDateTime.day}/${selectedDateTime.month}/${selectedDateTime.year}';
    return Scaffold(
      body: Center(
        child: Column(
          children: <Widget>[
            OutlinedButton(
              onPressed: () {
                _restorableDatePickerRouteFuture.present();
              },
              child: const Text('X'),
            ),
            Text(selectedDateTimeString),
          ],
        ),
      ),
    );
  }
}

class _DatePickerObserver extends NavigatorObserver {
  int datePickerCount = 0;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is DialogRoute) {
      datePickerCount++;
    }
    super.didPush(route, previousRoute);
  }

  @override
  void didPop(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is DialogRoute) {
      datePickerCount--;
    }
    super.didPop(route, previousRoute);
  }
}
