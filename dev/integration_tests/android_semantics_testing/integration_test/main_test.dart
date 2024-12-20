// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:android_semantics_testing/android_semantics_testing.dart';
import 'package:android_semantics_testing/main.dart' as app;
import 'package:android_semantics_testing/test_constants.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

// The accessibility focus actions are added when a semantics node receives or
// lose accessibility focus. This test ignores these actions since it is hard to
// predict which node has the accessibility focus after a screen changes.
const List<AndroidSemanticsAction> ignoredAccessibilityFocusActions = <AndroidSemanticsAction>[
  AndroidSemanticsAction.accessibilityFocus,
  AndroidSemanticsAction.clearAccessibilityFocus,
];

const MethodChannel kSemanticsChannel = MethodChannel('semantics');

Future<void> setClipboard(String message) async {
  final Completer<void> completer = Completer<void>();
  Future<void> completeSetClipboard([Object? _]) async {
    await kSemanticsChannel.invokeMethod<dynamic>('setClipboard', <String, dynamic>{
      'message': message,
    });
    completer.complete();
  }

  if (SchedulerBinding.instance.hasScheduledFrame) {
    SchedulerBinding.instance.addPostFrameCallback(completeSetClipboard);
  } else {
    completeSetClipboard();
  }
  await completer.future;
}

Future<AndroidSemanticsNode> getSemantics(Finder finder, WidgetTester tester) async {
  final int id = tester.getSemantics(finder).id;
  final Completer<String> completer = Completer<String>();
  Future<void> completeSemantics([Object? _]) async {
    final dynamic result = await kSemanticsChannel.invokeMethod<dynamic>(
      'getSemanticsNode',
      <String, dynamic>{'id': id},
    );
    completer.complete(json.encode(result));
  }

  if (SchedulerBinding.instance.hasScheduledFrame) {
    SchedulerBinding.instance.addPostFrameCallback(completeSemantics);
  } else {
    completeSemantics();
  }
  return AndroidSemanticsNode.deserialize(await completer.future);
}

Future<void> main() async {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  group('AccessibilityBridge', () {
    group('TextField', () {
      Future<void> prepareTextField(WidgetTester tester) async {
        app.main();
        await tester.pumpAndSettle();
        await tester.tap(find.text(textFieldRoute));
        await tester.pumpAndSettle();

        // The text selection menu and related semantics vary depending on if
        // the clipboard contents are pasteable. Copy some text into the
        // clipboard to make sure these tests always run with pasteable content
        // in the clipboard.
        // Ideally this should test the case where there is nothing on the
        // clipboard as well, but there is no reliable way to clear the
        // clipboard on Android devices.
        await setClipboard('Hello World');
        await tester.pumpAndSettle();
      }

      testWidgets('TextField has correct Android semantics', (WidgetTester tester) async {
        final Finder normalTextField = find.descendant(
          of: find.byKey(const ValueKey<String>(normalTextFieldKeyValue)),
          matching: find.byType(EditableText),
        );

        await prepareTextField(tester);
        expect(
          await getSemantics(normalTextField, tester),
          hasAndroidSemantics(
            className: AndroidClassName.editText,
            isEditable: true,
            isFocusable: true,
            isFocused: false,
            isPassword: false,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
            // We can't predict the a11y focus when the screen changes.
            ignoredActions: ignoredAccessibilityFocusActions,
          ),
        );
        await tester.tap(normalTextField);
        await tester.pumpAndSettle();

        expect(
          await getSemantics(normalTextField, tester),
          hasAndroidSemantics(
            className: AndroidClassName.editText,
            isFocusable: true,
            isFocused: true,
            isEditable: true,
            isPassword: false,
            actions: <AndroidSemanticsAction>[
              AndroidSemanticsAction.click,
              AndroidSemanticsAction.paste,
              AndroidSemanticsAction.setSelection,
              AndroidSemanticsAction.setText,
            ],
            // We can't predict the a11y focus when the screen changes.
            ignoredActions: ignoredAccessibilityFocusActions,
          ),
        );

        await tester.enterText(normalTextField, 'hello world');
        await tester.pumpAndSettle();

        expect(
          await getSemantics(normalTextField, tester),
          hasAndroidSemantics(
            text: 'hello world',
            className: AndroidClassName.editText,
            isFocusable: true,
            isFocused: true,
            isEditable: true,
            isPassword: false,
            actions: <AndroidSemanticsAction>[
              AndroidSemanticsAction.click,
              AndroidSemanticsAction.paste,
              AndroidSemanticsAction.setSelection,
              AndroidSemanticsAction.setText,
              AndroidSemanticsAction.previousAtMovementGranularity,
            ],
            // We can't predict the a11y focus when the screen changes.
            ignoredActions: ignoredAccessibilityFocusActions,
          ),
        );
      }, timeout: Timeout.none);

      testWidgets('password TextField has correct Android semantics', (WidgetTester tester) async {
        final Finder passwordTextField = find.descendant(
          of: find.byKey(const ValueKey<String>(passwordTextFieldKeyValue)),
          matching: find.byType(EditableText),
        );

        await prepareTextField(tester);
        expect(
          await getSemantics(passwordTextField, tester),
          hasAndroidSemantics(
            className: AndroidClassName.editText,
            isEditable: true,
            isFocusable: true,
            isFocused: false,
            isPassword: true,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
            // We can't predict the a11y focus when the screen changes.
            ignoredActions: ignoredAccessibilityFocusActions,
          ),
        );

        await tester.tap(passwordTextField);
        await tester.pumpAndSettle();

        expect(
          await getSemantics(passwordTextField, tester),
          hasAndroidSemantics(
            className: AndroidClassName.editText,
            isFocusable: true,
            isFocused: true,
            isEditable: true,
            isPassword: true,
            actions: <AndroidSemanticsAction>[
              AndroidSemanticsAction.click,
              AndroidSemanticsAction.paste,
              AndroidSemanticsAction.setSelection,
              AndroidSemanticsAction.setText,
            ],
            // We can't predict the a11y focus when the screen changes.
            ignoredActions: ignoredAccessibilityFocusActions,
          ),
        );

        await tester.enterText(passwordTextField, 'hello world');
        await tester.pumpAndSettle();

        expect(
          await getSemantics(passwordTextField, tester),
          hasAndroidSemantics(
            text: '\u{2022}' * ('hello world'.length),
            className: AndroidClassName.editText,
            isFocusable: true,
            isFocused: true,
            isEditable: true,
            isPassword: true,
            actions: <AndroidSemanticsAction>[
              AndroidSemanticsAction.click,
              AndroidSemanticsAction.paste,
              AndroidSemanticsAction.setSelection,
              AndroidSemanticsAction.setText,
              AndroidSemanticsAction.previousAtMovementGranularity,
            ],
            // We can't predict the a11y focus when the screen changes.
            ignoredActions: ignoredAccessibilityFocusActions,
          ),
        );
      }, timeout: Timeout.none);
    });

    group('SelectionControls', () {
      Future<void> prepareSelectionControls(WidgetTester tester) async {
        app.main();
        await tester.pumpAndSettle();
        await tester.tap(find.text(selectionControlsRoute));
        await tester.pumpAndSettle();
      }

      testWidgets('Checkbox has correct Android semantics', (WidgetTester tester) async {
        final Finder checkbox = find.byKey(const ValueKey<String>(checkboxKeyValue));
        final Finder disabledCheckbox = find.byKey(
          const ValueKey<String>(disabledCheckboxKeyValue),
        );

        await prepareSelectionControls(tester);
        expect(
          await getSemantics(checkbox, tester),
          hasAndroidSemantics(
            className: AndroidClassName.checkBox,
            isChecked: false,
            isCheckable: true,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );

        await tester.tap(checkbox);
        await tester.pumpAndSettle();

        expect(
          await getSemantics(checkbox, tester),
          hasAndroidSemantics(
            className: AndroidClassName.checkBox,
            isChecked: true,
            isCheckable: true,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );
        expect(
          await getSemantics(disabledCheckbox, tester),
          hasAndroidSemantics(
            className: AndroidClassName.checkBox,
            isCheckable: true,
            isEnabled: false,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: const <AndroidSemanticsAction>[],
          ),
        );
      }, timeout: Timeout.none);

      testWidgets('Radio has correct Android semantics', (WidgetTester tester) async {
        final Finder radio = find.byKey(const ValueKey<String>(radio2KeyValue));

        await prepareSelectionControls(tester);
        expect(
          await getSemantics(radio, tester),
          hasAndroidSemantics(
            className: AndroidClassName.radio,
            isChecked: false,
            isCheckable: true,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );

        await tester.tap(radio);
        await tester.pumpAndSettle();

        expect(
          await getSemantics(radio, tester),
          hasAndroidSemantics(
            className: AndroidClassName.radio,
            isChecked: true,
            isCheckable: true,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );
      }, timeout: Timeout.none);

      testWidgets('Switch has correct Android semantics', (WidgetTester tester) async {
        final Finder switchFinder = find.byKey(const ValueKey<String>(switchKeyValue));

        await prepareSelectionControls(tester);
        expect(
          await getSemantics(switchFinder, tester),
          hasAndroidSemantics(
            className: AndroidClassName.toggleSwitch,
            isChecked: false,
            isCheckable: true,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );

        await tester.tap(switchFinder);
        await tester.pumpAndSettle();

        expect(
          await getSemantics(switchFinder, tester),
          hasAndroidSemantics(
            className: AndroidClassName.toggleSwitch,
            isChecked: true,
            isCheckable: true,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );
      }, timeout: Timeout.none);

      // Regression test for https://github.com/flutter/flutter/issues/20820.
      testWidgets('Switch can be labeled', (WidgetTester tester) async {
        final Finder switchFinder = find.byKey(const ValueKey<String>(labeledSwitchKeyValue));

        await prepareSelectionControls(tester);
        expect(
          await getSemantics(switchFinder, tester),
          hasAndroidSemantics(
            className: AndroidClassName.toggleSwitch,
            isChecked: false,
            isCheckable: true,
            isEnabled: true,
            isFocusable: true,
            contentDescription: switchLabel,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );
      }, timeout: Timeout.none);
    });

    group('Popup Controls', () {
      Future<void> preparePopupControls(WidgetTester tester) async {
        app.main();
        await tester.pumpAndSettle();
        await tester.tap(find.text(popupControlsRoute));
        await tester.pumpAndSettle();
      }

      testWidgets('Popup Menu has correct Android semantics', (WidgetTester tester) async {
        final Finder popupButton = find.byKey(const ValueKey<String>(popupButtonKeyValue));

        await preparePopupControls(tester);
        expect(
          await getSemantics(popupButton, tester),
          hasAndroidSemantics(
            className: AndroidClassName.button,
            isChecked: false,
            isCheckable: false,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );

        await tester.tap(popupButton);
        await tester.pumpAndSettle();

        try {
          for (final String item in popupItems) {
            expect(
              await getSemantics(find.byKey(ValueKey<String>('$popupKeyValue.$item')), tester),
              hasAndroidSemantics(
                className: AndroidClassName.button,
                isChecked: false,
                isCheckable: false,
                isEnabled: true,
                isFocusable: true,
                ignoredActions: ignoredAccessibilityFocusActions,
                actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
              ),
              reason: "Popup $item doesn't have the right semantics",
            );
          }
          await tester.tap(find.byKey(ValueKey<String>('$popupKeyValue.${popupItems.first}')));
          await tester.pumpAndSettle();

          // Pop up the menu again, to verify that TalkBack gets the right answer
          // more than just the first time.
          await tester.tap(popupButton);
          await tester.pumpAndSettle();

          for (final String item in popupItems) {
            expect(
              await getSemantics(find.byKey(ValueKey<String>('$popupKeyValue.$item')), tester),
              hasAndroidSemantics(
                className: AndroidClassName.button,
                isChecked: false,
                isCheckable: false,
                isEnabled: true,
                isFocusable: true,
                ignoredActions: ignoredAccessibilityFocusActions,
                actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
              ),
              reason: "Popup $item doesn't have the right semantics the second time",
            );
          }
        } finally {
          await tester.tap(find.byKey(ValueKey<String>('$popupKeyValue.${popupItems.first}')));
        }
      }, timeout: Timeout.none);

      testWidgets('Dropdown Menu has correct Android semantics', (WidgetTester tester) async {
        final Finder dropdownButton = find.byKey(const ValueKey<String>(dropdownButtonKeyValue));

        await preparePopupControls(tester);
        expect(
          await getSemantics(dropdownButton, tester),
          hasAndroidSemantics(
            className: AndroidClassName.button,
            isChecked: false,
            isCheckable: false,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );

        await tester.tap(dropdownButton);
        await tester.pumpAndSettle();

        try {
          for (final String item in popupItems) {
            // There are two copies of each item, so we want to find the version
            // that is in the overlay, not the one in the dropdown.
            expect(
              await getSemantics(
                find.descendant(
                  of: find.byType(Scrollable),
                  matching: find.byKey(ValueKey<String>('$dropdownKeyValue.$item')),
                ),
                tester,
              ),
              hasAndroidSemantics(
                className: AndroidClassName.view,
                isChecked: false,
                isCheckable: false,
                isEnabled: true,
                isFocusable: true,
                ignoredActions: ignoredAccessibilityFocusActions,
                actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
              ),
              reason: "Dropdown $item doesn't have the right semantics",
            );
          }
          await tester.tap(
            find.descendant(
              of: find.byType(Scrollable),
              matching: find.byKey(ValueKey<String>('$dropdownKeyValue.${popupItems.first}')),
            ),
          );
          await tester.pumpAndSettle();

          // Pop up the dropdown again, to verify that TalkBack gets the right answer
          // more than just the first time.
          await tester.tap(dropdownButton);
          await tester.pumpAndSettle();

          for (final String item in popupItems) {
            // There are two copies of each item, so we want to find the version
            // that is in the overlay, not the one in the dropdown.
            expect(
              await getSemantics(
                find.descendant(
                  of: find.byType(Scrollable),
                  matching: find.byKey(ValueKey<String>('$dropdownKeyValue.$item')),
                ),
                tester,
              ),
              hasAndroidSemantics(
                className: AndroidClassName.view,
                isChecked: false,
                isCheckable: false,
                isEnabled: true,
                isFocusable: true,
                ignoredActions: ignoredAccessibilityFocusActions,
                actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
              ),
              reason: "Dropdown $item doesn't have the right semantics the second time.",
            );
          }
        } finally {
          await tester.tap(
            find.descendant(
              of: find.byType(Scrollable),
              matching: find.byKey(ValueKey<String>('$dropdownKeyValue.${popupItems.first}')),
            ),
          );
        }
      }, timeout: Timeout.none);

      testWidgets('Modal alert dialog has correct Android semantics', (WidgetTester tester) async {
        final Finder alertButton = find.byKey(const ValueKey<String>(alertButtonKeyValue));

        await preparePopupControls(tester);
        expect(
          await getSemantics(alertButton, tester),
          hasAndroidSemantics(
            className: AndroidClassName.button,
            isChecked: false,
            isCheckable: false,
            isEnabled: true,
            isFocusable: true,
            ignoredActions: ignoredAccessibilityFocusActions,
            actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
          ),
        );

        await tester.tap(alertButton);
        await tester.pumpAndSettle();

        try {
          expect(
            await getSemantics(find.byKey(const ValueKey<String>('$alertKeyValue.OK')), tester),
            hasAndroidSemantics(
              className: AndroidClassName.button,
              isChecked: false,
              isCheckable: false,
              isEnabled: true,
              isFocusable: true,
              ignoredActions: ignoredAccessibilityFocusActions,
              actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
            ),
            reason: "Alert OK button doesn't have the right semantics",
          );

          for (final String item in <String>['Title', 'Body1', 'Body2']) {
            expect(
              await getSemantics(find.byKey(ValueKey<String>('$alertKeyValue.$item')), tester),
              hasAndroidSemantics(
                className: AndroidClassName.view,
                isChecked: false,
                isCheckable: false,
                isEnabled: true,
                isFocusable: true,
                ignoredActions: ignoredAccessibilityFocusActions,
                actions: <AndroidSemanticsAction>[],
              ),
              reason: "Alert $item button doesn't have the right semantics",
            );
          }

          await tester.tap(find.byKey(const ValueKey<String>('$alertKeyValue.OK')));
          await tester.pumpAndSettle();

          // Pop up the alert again, to verify that TalkBack gets the right answer
          // more than just the first time.
          await tester.tap(alertButton);
          await tester.pumpAndSettle();

          expect(
            await getSemantics(find.byKey(const ValueKey<String>('$alertKeyValue.OK')), tester),
            hasAndroidSemantics(
              className: AndroidClassName.button,
              isChecked: false,
              isCheckable: false,
              isEnabled: true,
              isFocusable: true,
              ignoredActions: ignoredAccessibilityFocusActions,
              actions: <AndroidSemanticsAction>[AndroidSemanticsAction.click],
            ),
            reason: "Alert OK button doesn't have the right semantics",
          );

          for (final String item in <String>['Title', 'Body1', 'Body2']) {
            expect(
              await getSemantics(find.byKey(ValueKey<String>('$alertKeyValue.$item')), tester),
              hasAndroidSemantics(
                className: AndroidClassName.view,
                isChecked: false,
                isCheckable: false,
                isEnabled: true,
                isFocusable: true,
                ignoredActions: ignoredAccessibilityFocusActions,
                actions: <AndroidSemanticsAction>[],
              ),
              reason: "Alert $item button doesn't have the right semantics",
            );
          }
        } finally {
          await tester.tap(find.byKey(const ValueKey<String>('$alertKeyValue.OK')));
        }
      }, timeout: Timeout.none);
    });

    group('Headings', () {
      Future<void> prepareHeading(WidgetTester tester) async {
        app.main();
        await tester.pumpAndSettle();
        await tester.tap(find.text(headingsRoute));
        await tester.pumpAndSettle();
      }

      testWidgets('AppBar title has correct Android heading semantics', (
        WidgetTester tester,
      ) async {
        await prepareHeading(tester);
        expect(
          await getSemantics(find.byKey(const ValueKey<String>(appBarTitleKeyValue)), tester),
          hasAndroidSemantics(isHeading: true),
        );
      }, timeout: Timeout.none);

      testWidgets('body text does not have Android heading semantics', (WidgetTester tester) async {
        await prepareHeading(tester);
        expect(
          await getSemantics(find.byKey(const ValueKey<String>(bodyTextKeyValue)), tester),
          hasAndroidSemantics(isHeading: false),
        );
      }, timeout: Timeout.none);
    });
  });
}
