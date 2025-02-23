// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show TapGestureRecognizer;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

void main() => runApp(const MyApp());

class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  State<MyApp> createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      debugDumpSemanticsTree();
    });
  }

  @override
  Widget build(BuildContext context) {
    final TextStyle? bodyStyle = Theme.of(context).textTheme.bodyLarge;
    final TextStyle customStyle1 = Theme.of(
      context,
    ).textTheme.bodyMedium!.copyWith(color: Colors.blue);
    final TextStyle customStyle2 = Theme.of(
      context,
    ).textTheme.labelMedium!.copyWith(color: Colors.green);
    return MaterialApp(
      title: 'UI Testing with Text and RichText',
      home: Scaffold(
        body: SafeArea(
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16.0),
            child: Center(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  const SizedBox(height: 16.0),
                  Align(
                    child: Text(
                      'Demonstration of automation tools support in Semantics for Text and RichText',
                      style: Theme.of(context).textTheme.titleMedium,
                      textAlign: TextAlign.center,
                    ),
                  ),
                  const SizedBox(height: 16.0),
                  const Text(
                    'The identifier property in Semantics widget is used for UI testing with tools that work by querying the native accessibility, like UIAutomator, XCUITest, or Appium. It can be matched with CommonFinders.bySemanticsIdentifier.',
                  ),
                  const Divider(),
                  Text('Text Example:', style: bodyStyle),
                  const Text(
                    'This text has a custom label and an identifier. In Android, the label is used as the content-desc, and the identifier is used as the resource-id.',
                    semanticsLabel: 'This is a custom label',
                    // semanticsIdentifier:
                    //     'This is a custom identifier that only the automation tools are able to see',
                  ),
                  const Divider(),
                  Text('Text.rich Example:', style: bodyStyle),
                  Text.rich(
                    TextSpan(
                      text: 'This text contains both identifier and label.',
                      semanticsLabel: 'Custom label',
                      semanticsIdentifier: 'Custom identifier',
                      style: customStyle1,
                      children: <TextSpan>[
                        TextSpan(
                          text: ' While this one contains only label',
                          semanticsLabel: 'Hello world',
                          style: customStyle2,
                        ),
                        const TextSpan(
                          text: ' and this contains only identifier,',
                          semanticsIdentifier: 'Hello to the automation tool',
                        ),
                        TextSpan(
                          text: ' this text contains neither identifier nor label.',
                          style: customStyle2,
                        ),
                      ],
                    ),
                  ),
                  const Divider(),
                  Text('Multi-tenant Example:', style: bodyStyle),
                  const SizedBox(height: 16),
                  Column(
                    spacing: 16.0,
                    children: <Widget>[
                      Center(
                        child: Text.rich(
                          TextSpan(
                            text: 'Please open the ',
                            // semanticsIdentifier: 'please_open',
                            children: <InlineSpan>[
                              const TextSpan(
                                text: 'product 1',
                                // semanticsIdentifier: 'product_name',
                              ),
                              const TextSpan(
                                text: '\nto use this app.',
                                // semanticsIdentifier: 'to_use_app',
                              ),
                              TextSpan(
                                text: ' Learn more',
                                // semanticsIdentifier: 'learn_more_link',
                                style: const TextStyle(color: Colors.blue),
                                recognizer:
                                    TapGestureRecognizer()
                                      ..onTap = () {
                                        print('Learn more');
                                      },
                              ),
                            ],
                          ),
                          textAlign: TextAlign.center,
                        ),
                      ),
                      Center(
                        child: Text.rich(
                          TextSpan(
                            text: 'Please open the ',
                            // semanticsIdentifier: 'please_open',
                            children: <InlineSpan>[
                              const TextSpan(
                                text: 'product 2',
                                // semanticsIdentifier: 'product_name',
                              ),
                              const TextSpan(
                                text: '\nto access this app.',
                                // semanticsIdentifier: 'to_use_app',
                              ),
                              TextSpan(
                                text: ' Find out more',
                                // semanticsIdentifier: 'learn_more_link',
                                style: const TextStyle(color: Colors.blue),
                                recognizer:
                                    TapGestureRecognizer()
                                      ..onTap = () {
                                        print('Learn more');
                                      },
                              ),
                            ],
                          ),
                          textAlign: TextAlign.center,
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}
