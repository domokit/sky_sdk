// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This sample demonstrates how to use a PopScope to wrap a widget that
// may pop the page with a result.

import 'package:flutter/material.dart';

void main() => runApp(const NavigatorPopHandlerApp());

class NavigatorPopHandlerApp extends StatelessWidget {
  const NavigatorPopHandlerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      initialRoute: '/home',
      onGenerateRoute: (RouteSettings settings) {
        return switch (settings.name) {
          '/two' => MaterialPageRoute<FormData>(
            builder: (BuildContext context) => const _PageTwo(),
          ),
          _ => MaterialPageRoute<void>(builder: (BuildContext context) => const _HomePage()),
        };
      },
    );
  }
}

class _HomePage extends StatefulWidget {
  const _HomePage();

  @override
  State<_HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<_HomePage> {
  FormData? _formData;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Page One'),
            if (_formData != null)
              Text('Hello ${_formData!.name}, whose favorite food is ${_formData!.favoriteFood}.'),
            TextButton(
              onPressed: () async {
                final FormData formData =
                    await Navigator.of(context).pushNamed<FormData?>('/two') ?? const FormData();
                if (formData != _formData) {
                  setState(() {
                    _formData = formData;
                  });
                }
              },
              child: const Text('Next page'),
            ),
          ],
        ),
      ),
    );
  }
}

class _PopScopeWrapper extends StatelessWidget {
  const _PopScopeWrapper({required this.child});

  final Widget child;

  Future<bool?> _showBackDialog(BuildContext context) {
    return showDialog<bool>(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: const Text('Are you sure?'),
          content: const Text('Are you sure you want to leave this page?'),
          actions: <Widget>[
            TextButton(
              style: TextButton.styleFrom(textStyle: Theme.of(context).textTheme.labelLarge),
              child: const Text('Never mind'),
              onPressed: () {
                Navigator.pop(context, false);
              },
            ),
            TextButton(
              style: TextButton.styleFrom(textStyle: Theme.of(context).textTheme.labelLarge),
              child: const Text('Leave'),
              onPressed: () {
                Navigator.pop(context, true);
              },
            ),
          ],
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return PopScope<FormData>(
      canPop: false,
      // The result argument contains the pop result that is defined in `_PageTwo`.
      onPopInvokedWithResult: (bool didPop, FormData? result) async {
        if (didPop) {
          return;
        }
        final bool shouldPop = await _showBackDialog(context) ?? false;
        if (context.mounted && shouldPop) {
          Navigator.pop(context, result);
        }
      },
      child: child,
    );
  }
}

// This is a PopScope wrapper over _PageTwoBody
class _PageTwo extends StatelessWidget {
  const _PageTwo();

  @override
  Widget build(BuildContext context) {
    return const _PopScopeWrapper(child: _PageTwoBody());
  }
}

class _PageTwoBody extends StatefulWidget {
  const _PageTwoBody();

  @override
  State<_PageTwoBody> createState() => _PageTwoBodyState();
}

class _PageTwoBodyState extends State<_PageTwoBody> {
  FormData _formData = const FormData();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Page Two'),
            Form(
              child: Column(
                children: <Widget>[
                  TextFormField(
                    decoration: const InputDecoration(hintText: 'Enter your name.'),
                    onChanged: (String value) {
                      _formData = _formData.copyWith(name: value);
                    },
                  ),
                  TextFormField(
                    decoration: const InputDecoration(hintText: 'Enter your favorite food.'),
                    onChanged: (String value) {
                      _formData = _formData.copyWith(favoriteFood: value);
                    },
                  ),
                ],
              ),
            ),
            TextButton(
              onPressed: () async {
                Navigator.maybePop(context, _formData);
              },
              child: const Text('Go back'),
            ),
          ],
        ),
      ),
    );
  }
}

@immutable
class FormData {
  const FormData({this.name = '', this.favoriteFood = ''});

  final String name;
  final String favoriteFood;

  FormData copyWith({String? name, String? favoriteFood}) {
    return FormData(name: name ?? this.name, favoriteFood: favoriteFood ?? this.favoriteFood);
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is FormData && other.name == name && other.favoriteFood == favoriteFood;
  }

  @override
  int get hashCode => Object.hash(name, favoriteFood);
}
