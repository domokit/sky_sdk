// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/material.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import '../../layout/adaptive.dart';
import '../../layout/image_placeholder.dart';
import '../../layout/letter_spacing.dart';
import '../../layout/text_scale.dart';
import 'app.dart';
import 'theme.dart';

const double _horizontalPadding = 24.0;

double desktopLoginScreenMainAreaWidth({required BuildContext context}) {
  return min(
    360 * reducedTextScale(context),
    MediaQuery.of(context).size.width - 2 * _horizontalPadding,
  );
}

class LoginPage extends StatelessWidget {
  const LoginPage({super.key});

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);

    return ApplyTextOptions(
      child:
          isDesktop
              ? LayoutBuilder(
                builder:
                    (BuildContext context, BoxConstraints constraints) => Scaffold(
                      body: SafeArea(
                        child: Center(
                          child: SizedBox(
                            width: desktopLoginScreenMainAreaWidth(context: context),
                            child: const Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: <Widget>[
                                _ShrineLogo(),
                                SizedBox(height: 40),
                                _UsernameTextField(),
                                SizedBox(height: 16),
                                _PasswordTextField(),
                                SizedBox(height: 24),
                                _CancelAndNextButtons(),
                                SizedBox(height: 62),
                              ],
                            ),
                          ),
                        ),
                      ),
                    ),
              )
              : Scaffold(
                body: SafeArea(
                  child: ListView(
                    restorationId: 'login_list_view',
                    physics: const ClampingScrollPhysics(),
                    padding: const EdgeInsets.symmetric(horizontal: _horizontalPadding),
                    children: const <Widget>[
                      SizedBox(height: 80),
                      _ShrineLogo(),
                      SizedBox(height: 120),
                      _UsernameTextField(),
                      SizedBox(height: 12),
                      _PasswordTextField(),
                      _CancelAndNextButtons(),
                    ],
                  ),
                ),
              ),
    );
  }
}

class _ShrineLogo extends StatelessWidget {
  const _ShrineLogo();

  @override
  Widget build(BuildContext context) {
    return ExcludeSemantics(
      child: Column(
        children: <Widget>[
          const FadeInImagePlaceholder(
            image: AssetImage('packages/shrine_images/diamond.png'),
            placeholder: SizedBox(width: 34, height: 34),
          ),
          const SizedBox(height: 16),
          Text('SHRINE', style: Theme.of(context).textTheme.headlineSmall),
        ],
      ),
    );
  }
}

class _UsernameTextField extends StatelessWidget {
  const _UsernameTextField();

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;

    return TextField(
      textInputAction: TextInputAction.next,
      restorationId: 'username_text_field',
      cursorColor: colorScheme.onSurface,
      decoration: InputDecoration(
        labelText: GalleryLocalizations.of(context)!.shrineLoginUsernameLabel,
        labelStyle: TextStyle(letterSpacing: letterSpacingOrNone(mediumLetterSpacing)),
      ),
    );
  }
}

class _PasswordTextField extends StatelessWidget {
  const _PasswordTextField();

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;

    return TextField(
      restorationId: 'password_text_field',
      cursorColor: colorScheme.onSurface,
      obscureText: true,
      decoration: InputDecoration(
        labelText: GalleryLocalizations.of(context)!.shrineLoginPasswordLabel,
        labelStyle: TextStyle(letterSpacing: letterSpacingOrNone(mediumLetterSpacing)),
      ),
    );
  }
}

class _CancelAndNextButtons extends StatelessWidget {
  const _CancelAndNextButtons();

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;

    final bool isDesktop = isDisplayDesktop(context);

    final EdgeInsets buttonTextPadding =
        isDesktop ? const EdgeInsets.symmetric(horizontal: 24, vertical: 16) : EdgeInsets.zero;

    return Padding(
      padding: isDesktop ? EdgeInsets.zero : const EdgeInsets.all(8),
      child: OverflowBar(
        spacing: isDesktop ? 0 : 8,
        alignment: MainAxisAlignment.end,
        children: <Widget>[
          TextButton(
            style: TextButton.styleFrom(
              shape: const BeveledRectangleBorder(
                borderRadius: BorderRadius.all(Radius.circular(7)),
              ),
            ),
            onPressed: () {
              // The login screen is immediately displayed on top of
              // the Shrine home screen using onGenerateRoute and so
              // rootNavigator must be set to true in order to get out
              // of Shrine completely.
              Navigator.of(context, rootNavigator: true).pop();
            },
            child: Padding(
              padding: buttonTextPadding,
              child: Text(
                GalleryLocalizations.of(context)!.shrineCancelButtonCaption,
                style: TextStyle(color: colorScheme.onSurface),
              ),
            ),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(
              elevation: 8,
              shape: const BeveledRectangleBorder(
                borderRadius: BorderRadius.all(Radius.circular(7)),
              ),
            ),
            onPressed: () {
              Navigator.of(context).restorablePushNamed(ShrineApp.homeRoute);
            },
            child: Padding(
              padding: buttonTextPadding,
              child: Text(
                GalleryLocalizations.of(context)!.shrineNextButtonCaption,
                style: TextStyle(letterSpacing: letterSpacingOrNone(largeLetterSpacing)),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
