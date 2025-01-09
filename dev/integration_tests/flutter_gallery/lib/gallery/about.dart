// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart' show defaultTargetPlatform;
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:url_launcher/url_launcher.dart';

class _LinkTextSpan extends TextSpan {
  // Beware!
  //
  // This class is only safe because the TapGestureRecognizer is not
  // given a deadline and therefore never allocates any resources.
  //
  // In any other situation -- setting a deadline, using any of the less trivial
  // recognizers, etc -- you would have to manage the gesture recognizer's
  // lifetime and call dispose() when the TextSpan was no longer being rendered.
  //
  // Since TextSpan itself is @immutable, this means that you would have to
  // manage the recognizer from outside the TextSpan, e.g. in the State of a
  // stateful widget that then hands the recognizer to the TextSpan.

  _LinkTextSpan({super.style, required String url, String? text})
    : super(
        text: text ?? url,
        recognizer:
            TapGestureRecognizer()
              ..onTap = () {
                launchUrl(Uri.parse(url), mode: LaunchMode.externalApplication);
              },
      );
}

void showGalleryAboutDialog(BuildContext context) {
  final ThemeData themeData = Theme.of(context);
  final TextStyle? aboutTextStyle = themeData.textTheme.bodyLarge;
  final TextStyle linkStyle = themeData.textTheme.bodyLarge!.copyWith(
    color: themeData.colorScheme.primary,
  );

  showAboutDialog(
    context: context,
    applicationVersion: 'January 2019',
    applicationIcon: const FlutterLogo(),
    applicationLegalese: '© 2014 The Flutter Authors',
    children: <Widget>[
      Padding(
        padding: const EdgeInsets.only(top: 24.0),
        child: RichText(
          text: TextSpan(
            children: <TextSpan>[
              TextSpan(
                style: aboutTextStyle,
                text:
                    'Flutter is an open-source project to help developers '
                    'build high-performance, high-fidelity, mobile apps for '
                    '${defaultTargetPlatform == TargetPlatform.iOS ? 'multiple platforms' : 'iOS and Android'} '
                    'from a single codebase. This design lab is a playground '
                    "and showcase of Flutter's many widgets, behaviors, "
                    'animations, layouts, and more. Learn more about Flutter at ',
              ),
              _LinkTextSpan(style: linkStyle, url: 'https://flutter.dev'),
              TextSpan(
                style: aboutTextStyle,
                text: '.\n\nTo see the source code for this app, please visit the ',
              ),
              _LinkTextSpan(
                style: linkStyle,
                url: 'https://goo.gl/iv1p4G',
                text: 'flutter github repo',
              ),
              TextSpan(style: aboutTextStyle, text: '.'),
            ],
          ),
        ),
      ),
    ],
  );
}
