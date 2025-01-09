// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/autofill/autofill_group.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AutofillGroupExample renders and updates correctly', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutofillGroupExampleApp());

    expect(find.text('Shipping address'), findsOneWidget);
    expect(find.text('Billing address'), findsOneWidget);
    expect(find.byType(TextField), findsNWidgets(5));
    expect(find.byType(Checkbox), findsOneWidget);
    expect(find.text('Contact Phone Number'), findsOneWidget);

    final Checkbox checkbox = tester.widget(find.byType(Checkbox));
    expect(checkbox.value, isTrue);

    // Tap to uncheck the checkbox (to show billing address fields).
    await tester.tap(find.byType(Checkbox));
    await tester.pump();

    expect(
      find.byType(AutofillGroup),
      findsNWidgets(3), // 3 AutofillGroup widgets created
    );

    expect(
      find.byType(TextField),
      findsNWidgets(7), // 5 initial + 2 billing address fields
    );

    final TextField shippingAddress1 = tester.widget(find.byType(TextField).at(0));
    expect(shippingAddress1.autofillHints, contains(AutofillHints.streetAddressLine1));

    final TextField shippingAddress2 = tester.widget(find.byType(TextField).at(1));
    expect(shippingAddress2.autofillHints, contains(AutofillHints.streetAddressLine2));

    final TextField billingAddress1 = tester.widget(find.byType(TextField).at(2));
    expect(billingAddress1.autofillHints, contains(AutofillHints.streetAddressLine1));

    final TextField billingAddress2 = tester.widget(find.byType(TextField).at(3));
    expect(billingAddress2.autofillHints, contains(AutofillHints.streetAddressLine2));

    // Credit card information fields.
    final TextField creditCardNumber = tester.widget(find.byType(TextField).at(4));
    expect(creditCardNumber.autofillHints, contains(AutofillHints.creditCardNumber));

    final TextField creditCardSecurityCode = tester.widget(find.byType(TextField).at(5));
    expect(creditCardSecurityCode.autofillHints, contains(AutofillHints.creditCardSecurityCode));

    // Contact phone number field.
    final TextField phoneNumber = tester.widget(find.byType(TextField).at(6));
    expect(phoneNumber.autofillHints, contains(AutofillHints.telephoneNumber));

    await tester.tap(find.byType(Checkbox));
    await tester.pump();

    expect(find.byType(TextField), findsNWidgets(5));
  });
}
