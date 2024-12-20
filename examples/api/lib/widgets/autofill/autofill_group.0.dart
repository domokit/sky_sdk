// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [AutofillGroup].

void main() => runApp(const AutofillGroupExampleApp());

class AutofillGroupExampleApp extends StatelessWidget {
  const AutofillGroupExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('AutofillGroup Sample')),
        body: const AutofillGroupExample(),
      ),
    );
  }
}

class AutofillGroupExample extends StatefulWidget {
  const AutofillGroupExample({super.key});

  @override
  State<AutofillGroupExample> createState() => _AutofillGroupExampleState();
}

class _AutofillGroupExampleState extends State<AutofillGroupExample> {
  bool isSameAddress = true;
  final TextEditingController shippingAddress1 = TextEditingController();
  final TextEditingController shippingAddress2 = TextEditingController();
  final TextEditingController billingAddress1 = TextEditingController();
  final TextEditingController billingAddress2 = TextEditingController();

  final TextEditingController creditCardNumber = TextEditingController();
  final TextEditingController creditCardSecurityCode = TextEditingController();

  final TextEditingController phoneNumber = TextEditingController();

  @override
  Widget build(BuildContext context) {
    return ListView(
      children: <Widget>[
        const Text('Shipping address'),
        // The address fields are grouped together as some platforms are
        // capable of autofilling all of these fields in one go.
        AutofillGroup(
          child: Column(
            children: <Widget>[
              TextField(
                controller: shippingAddress1,
                autofillHints: const <String>[AutofillHints.streetAddressLine1],
              ),
              TextField(
                controller: shippingAddress2,
                autofillHints: const <String>[AutofillHints.streetAddressLine2],
              ),
            ],
          ),
        ),
        const Text('Billing address'),
        Checkbox(
          value: isSameAddress,
          onChanged: (bool? newValue) {
            if (newValue != null) {
              setState(() {
                isSameAddress = newValue;
              });
            }
          },
        ),
        // Again the address fields are grouped together for the same reason.
        if (!isSameAddress)
          AutofillGroup(
            child: Column(
              children: <Widget>[
                TextField(
                  controller: billingAddress1,
                  autofillHints: const <String>[AutofillHints.streetAddressLine1],
                ),
                TextField(
                  controller: billingAddress2,
                  autofillHints: const <String>[AutofillHints.streetAddressLine2],
                ),
              ],
            ),
          ),
        const Text('Credit Card Information'),
        // The credit card number and the security code are grouped together
        // as some platforms are capable of autofilling both fields.
        AutofillGroup(
          child: Column(
            children: <Widget>[
              TextField(
                controller: creditCardNumber,
                autofillHints: const <String>[AutofillHints.creditCardNumber],
              ),
              TextField(
                controller: creditCardSecurityCode,
                autofillHints: const <String>[AutofillHints.creditCardSecurityCode],
              ),
            ],
          ),
        ),
        const Text('Contact Phone Number'),
        // The phone number field can still be autofilled despite lacking an
        // `AutofillScope`.
        TextField(
          controller: phoneNumber,
          autofillHints: const <String>[AutofillHints.telephoneNumber],
        ),
      ],
    );
  }
}
