// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [TextFormField].

const Duration kFakeHttpRequestDuration = Duration(seconds: 3);

void main() => runApp(const TextFormFieldExampleApp());

class TextFormFieldExampleApp extends StatelessWidget {
  const TextFormFieldExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: TextFormFieldExample());
  }
}

class TextFormFieldExample extends StatefulWidget {
  const TextFormFieldExample({super.key});

  @override
  State<TextFormFieldExample> createState() => _TextFormFieldExampleState();
}

class _TextFormFieldExampleState extends State<TextFormFieldExample> {
  final TextEditingController controller = TextEditingController();
  final GlobalKey<FormState> formKey = GlobalKey<FormState>();
  String? forceErrorText;
  bool isLoading = false;

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  String? validator(String? value) {
    if (value == null || value.isEmpty) {
      return 'This field is required';
    }
    if (value.length != value.replaceAll(' ', '').length) {
      return 'Username must not contain any spaces';
    }
    if (int.tryParse(value[0]) != null) {
      return 'Username must not start with a number';
    }
    if (value.length <= 2) {
      return 'Username should be at least 3 characters long';
    }
    return null;
  }

  void onChanged(String value) {
    // Nullify forceErrorText if the input changed.
    if (forceErrorText != null) {
      setState(() {
        forceErrorText = null;
      });
    }
  }

  Future<void> onSave() async {
    // Providing a default value in case this was called on the
    // first frame, the [fromKey.currentState] will be null.
    final bool isValid = formKey.currentState?.validate() ?? false;
    if (!isValid) {
      return;
    }

    setState(() => isLoading = true);
    final String? errorText = await validateUsernameFromServer(controller.text);

    if (context.mounted) {
      setState(() => isLoading = false);

      if (errorText != null) {
        setState(() {
          forceErrorText = errorText;
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Material(
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 24.0),
        child: Center(
          child: Form(
            key: formKey,
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                TextFormField(
                  forceErrorText: forceErrorText,
                  controller: controller,
                  decoration: const InputDecoration(hintText: 'Please write a username'),
                  validator: validator,
                  onChanged: onChanged,
                ),
                const SizedBox(height: 40.0),
                if (isLoading)
                  const CircularProgressIndicator()
                else
                  TextButton(onPressed: onSave, child: const Text('Save')),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

Future<String?> validateUsernameFromServer(String username) async {
  final Set<String> takenUsernames = <String>{'jack', 'alex'};

  await Future<void>.delayed(kFakeHttpRequestDuration);

  final bool isValid = !takenUsernames.contains(username);
  if (isValid) {
    return null;
  }

  return 'Username $username is already taken';
}
