// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../../gallery/demo.dart';

class TextFormFieldDemo extends StatefulWidget {
  const TextFormFieldDemo({super.key});

  static const String routeName = '/material/text-form-field';

  @override
  TextFormFieldDemoState createState() => TextFormFieldDemoState();
}

class PersonData {
  String? name = '';
  String? phoneNumber = '';
  String? email = '';
  String password = '';
}

class PasswordField extends StatefulWidget {
  const PasswordField({
    super.key,
    this.fieldKey,
    this.hintText,
    this.labelText,
    this.helperText,
    this.onSaved,
    this.validator,
    this.onFieldSubmitted,
  });

  final Key? fieldKey;
  final String? hintText;
  final String? labelText;
  final String? helperText;
  final FormFieldSetter<String>? onSaved;
  final FormFieldValidator<String>? validator;
  final ValueChanged<String>? onFieldSubmitted;

  @override
  State<PasswordField> createState() => _PasswordFieldState();
}

class _PasswordFieldState extends State<PasswordField> {
  bool _obscureText = true;

  @override
  Widget build(BuildContext context) {
    return TextFormField(
      key: widget.fieldKey,
      obscureText: _obscureText,
      maxLength: 8,
      onSaved: widget.onSaved,
      validator: widget.validator,
      onFieldSubmitted: widget.onFieldSubmitted,
      decoration: InputDecoration(
        border: const UnderlineInputBorder(),
        filled: true,
        hintText: widget.hintText,
        labelText: widget.labelText,
        helperText: widget.helperText,
        suffixIcon: GestureDetector(
          dragStartBehavior: DragStartBehavior.down,
          onTap: () {
            setState(() {
              _obscureText = !_obscureText;
            });
          },
          child: Icon(
            _obscureText ? Icons.visibility : Icons.visibility_off,
            semanticLabel: _obscureText ? 'show password' : 'hide password',
          ),
        ),
      ),
    );
  }
}

class TextFormFieldDemoState extends State<TextFormFieldDemo> {
  PersonData person = PersonData();

  void showInSnackBar(String value) {
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(value)));
  }

  AutovalidateMode _autovalidateMode = AutovalidateMode.disabled;
  bool _formWasEdited = false;

  final GlobalKey<FormState> _formKey = GlobalKey<FormState>();
  final GlobalKey<FormFieldState<String>> _passwordFieldKey = GlobalKey<FormFieldState<String>>();
  final _UsNumberTextInputFormatter _phoneNumberFormatter = _UsNumberTextInputFormatter();
  void _handleSubmitted() {
    final FormState form = _formKey.currentState!;
    if (!form.validate()) {
      _autovalidateMode = AutovalidateMode.always; // Start validating on every change.
      showInSnackBar('Please fix the errors in red before submitting.');
    } else {
      form.save();
      showInSnackBar("${person.name}'s phone number is ${person.phoneNumber}");
    }
  }

  String? _validateName(String? value) {
    _formWasEdited = true;
    if (value!.isEmpty) {
      return 'Name is required.';
    }
    final RegExp nameExp = RegExp(r'^[A-Za-z ]+$');
    if (!nameExp.hasMatch(value)) {
      return 'Please enter only alphabetical characters.';
    }
    return null;
  }

  String? _validatePhoneNumber(String? value) {
    _formWasEdited = true;
    final RegExp phoneExp = RegExp(r'^\(\d\d\d\) \d\d\d\-\d\d\d\d$');
    if (!phoneExp.hasMatch(value!)) {
      return '(###) ###-#### - Enter a US phone number.';
    }
    return null;
  }

  String? _validatePassword(String? value) {
    _formWasEdited = true;
    final FormFieldState<String> passwordField = _passwordFieldKey.currentState!;
    if (passwordField.value == null || passwordField.value!.isEmpty) {
      return 'Please enter a password.';
    }
    if (passwordField.value != value) {
      return "The passwords don't match";
    }
    return null;
  }

  Future<void> _handlePopInvoked(bool didPop, Object? result) async {
    if (didPop) {
      return;
    }

    final bool? result = await showDialog<bool>(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: const Text('This form has errors'),
          content: const Text('Really leave this form?'),
          actions: <Widget>[
            TextButton(
              child: const Text('YES'),
              onPressed: () {
                Navigator.of(context).pop(true);
              },
            ),
            TextButton(
              child: const Text('NO'),
              onPressed: () {
                Navigator.of(context).pop(false);
              },
            ),
          ],
        );
      },
    );

    if (result ?? false) {
      // Since this is the root route, quit the app where possible by invoking
      // the SystemNavigator. If this wasn't the root route, then
      // Navigator.maybePop could be used instead.
      // See https://github.com/flutter/flutter/issues/11490
      SystemNavigator.pop();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawerDragStartBehavior: DragStartBehavior.down,
      appBar: AppBar(
        title: const Text('Text fields'),
        actions: <Widget>[MaterialDemoDocumentationButton(TextFormFieldDemo.routeName)],
      ),
      body: SafeArea(
        top: false,
        bottom: false,
        child: Form(
          key: _formKey,
          autovalidateMode: _autovalidateMode,
          canPop:
              _formKey.currentState == null || !_formWasEdited || _formKey.currentState!.validate(),
          onPopInvokedWithResult: _handlePopInvoked,
          child: Scrollbar(
            child: SingleChildScrollView(
              primary: true,
              dragStartBehavior: DragStartBehavior.down,
              padding: const EdgeInsets.symmetric(horizontal: 16.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  const SizedBox(height: 24.0),
                  TextFormField(
                    textCapitalization: TextCapitalization.words,
                    decoration: const InputDecoration(
                      border: UnderlineInputBorder(),
                      filled: true,
                      icon: Icon(Icons.person),
                      hintText: 'What do people call you?',
                      labelText: 'Name *',
                    ),
                    onSaved: (String? value) {
                      person.name = value;
                    },
                    validator: _validateName,
                  ),
                  const SizedBox(height: 24.0),
                  TextFormField(
                    decoration: const InputDecoration(
                      border: UnderlineInputBorder(),
                      filled: true,
                      icon: Icon(Icons.phone),
                      hintText: 'Where can we reach you?',
                      labelText: 'Phone Number *',
                      prefixText: '+1',
                    ),
                    keyboardType: TextInputType.phone,
                    onSaved: (String? value) {
                      person.phoneNumber = value;
                    },
                    validator: _validatePhoneNumber,
                    // TextInputFormatters are applied in sequence.
                    inputFormatters: <TextInputFormatter>[
                      FilteringTextInputFormatter.digitsOnly,
                      // Fit the validating format.
                      _phoneNumberFormatter,
                    ],
                  ),
                  const SizedBox(height: 24.0),
                  TextFormField(
                    decoration: const InputDecoration(
                      border: UnderlineInputBorder(),
                      filled: true,
                      icon: Icon(Icons.email),
                      hintText: 'Your email address',
                      labelText: 'E-mail',
                    ),
                    keyboardType: TextInputType.emailAddress,
                    onSaved: (String? value) {
                      person.email = value;
                    },
                  ),
                  const SizedBox(height: 24.0),
                  TextFormField(
                    decoration: const InputDecoration(
                      border: OutlineInputBorder(),
                      hintText:
                          'Tell us about yourself (e.g., write down what you do or what hobbies you have)',
                      helperText: 'Keep it short, this is just a demo.',
                      labelText: 'Life story',
                    ),
                    maxLines: 3,
                  ),
                  const SizedBox(height: 24.0),
                  TextFormField(
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(
                      border: OutlineInputBorder(),
                      labelText: 'Salary',
                      prefixText: r'$',
                      suffixText: 'USD',
                      suffixStyle: TextStyle(color: Colors.green),
                    ),
                  ),
                  const SizedBox(height: 24.0),
                  PasswordField(
                    fieldKey: _passwordFieldKey,
                    helperText: 'No more than 8 characters.',
                    labelText: 'Password *',
                    onFieldSubmitted: (String value) {
                      setState(() {
                        person.password = value;
                      });
                    },
                  ),
                  const SizedBox(height: 24.0),
                  TextFormField(
                    enabled: person.password.isNotEmpty,
                    decoration: const InputDecoration(
                      border: UnderlineInputBorder(),
                      filled: true,
                      labelText: 'Re-type password',
                    ),
                    maxLength: 8,
                    obscureText: true,
                    validator: _validatePassword,
                  ),
                  const SizedBox(height: 24.0),
                  Center(
                    child: ElevatedButton(onPressed: _handleSubmitted, child: const Text('SUBMIT')),
                  ),
                  const SizedBox(height: 24.0),
                  Text('* indicates required field', style: Theme.of(context).textTheme.bodySmall),
                  const SizedBox(height: 24.0),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

/// Format incoming numeric text to fit the format of (###) ###-#### ##...
class _UsNumberTextInputFormatter extends TextInputFormatter {
  @override
  TextEditingValue formatEditUpdate(TextEditingValue oldValue, TextEditingValue newValue) {
    final int newTextLength = newValue.text.length;
    int selectionIndex = newValue.selection.end;
    int usedSubstringIndex = 0;
    final StringBuffer newText = StringBuffer();
    if (newTextLength >= 1) {
      newText.write('(');
      if (newValue.selection.end >= 1) {
        selectionIndex++;
      }
    }
    if (newTextLength >= 4) {
      final String value = newValue.text.substring(0, usedSubstringIndex = 3);
      newText.write('$value) ');
      if (newValue.selection.end >= 3) {
        selectionIndex += 2;
      }
    }
    if (newTextLength >= 7) {
      final String value = newValue.text.substring(3, usedSubstringIndex = 6);
      newText.write('$value-');
      if (newValue.selection.end >= 6) {
        selectionIndex++;
      }
    }
    if (newTextLength >= 11) {
      final String value = newValue.text.substring(6, usedSubstringIndex = 10);
      newText.write('$value ');
      if (newValue.selection.end >= 10) {
        selectionIndex++;
      }
    }
    // Dump the rest.
    if (newTextLength >= usedSubstringIndex) {
      newText.write(newValue.text.substring(usedSubstringIndex));
    }
    return TextEditingValue(
      text: newText.toString(),
      selection: TextSelection.collapsed(offset: selectionIndex),
    );
  }
}
