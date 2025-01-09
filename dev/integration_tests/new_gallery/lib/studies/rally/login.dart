// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import '../../layout/adaptive.dart';
import '../../layout/image_placeholder.dart';
import '../../layout/text_scale.dart';
import 'app.dart';
import 'colors.dart';

class LoginPage extends StatefulWidget {
  const LoginPage({super.key});

  @override
  State<LoginPage> createState() => _LoginPageState();
}

class _LoginPageState extends State<LoginPage> with RestorationMixin {
  final RestorableTextEditingController _usernameController = RestorableTextEditingController();
  final RestorableTextEditingController _passwordController = RestorableTextEditingController();

  @override
  String get restorationId => 'login_page';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_usernameController, restorationId);
    registerForRestoration(_passwordController, restorationId);
  }

  @override
  Widget build(BuildContext context) {
    return ApplyTextOptions(
      child: Scaffold(
        body: SafeArea(
          child: _MainView(
            usernameController: _usernameController.value,
            passwordController: _passwordController.value,
          ),
        ),
      ),
    );
  }

  @override
  void dispose() {
    _usernameController.dispose();
    _passwordController.dispose();
    super.dispose();
  }
}

class _MainView extends StatelessWidget {
  const _MainView({this.usernameController, this.passwordController});

  final TextEditingController? usernameController;
  final TextEditingController? passwordController;

  void _login(BuildContext context) {
    Navigator.of(context).restorablePushNamed(RallyApp.homeRoute);
  }

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    List<Widget> listViewChildren;

    if (isDesktop) {
      final double desktopMaxWidth = 400.0 + 100.0 * (cappedTextScale(context) - 1);
      listViewChildren = <Widget>[
        _UsernameInput(maxWidth: desktopMaxWidth, usernameController: usernameController),
        const SizedBox(height: 12),
        _PasswordInput(maxWidth: desktopMaxWidth, passwordController: passwordController),
        _LoginButton(
          maxWidth: desktopMaxWidth,
          onTap: () {
            _login(context);
          },
        ),
      ];
    } else {
      listViewChildren = <Widget>[
        const _SmallLogo(),
        _UsernameInput(usernameController: usernameController),
        const SizedBox(height: 12),
        _PasswordInput(passwordController: passwordController),
        _ThumbButton(
          onTap: () {
            _login(context);
          },
        ),
      ];
    }

    return Column(
      children: <Widget>[
        if (isDesktop) const _TopBar(),
        Expanded(
          child: Align(
            alignment: isDesktop ? Alignment.center : Alignment.topCenter,
            child: ListView(
              restorationId: 'login_list_view',
              shrinkWrap: true,
              padding: const EdgeInsets.symmetric(horizontal: 24),
              children: listViewChildren,
            ),
          ),
        ),
      ],
    );
  }
}

class _TopBar extends StatelessWidget {
  const _TopBar();

  @override
  Widget build(BuildContext context) {
    const SizedBox spacing = SizedBox(width: 30);
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Container(
      width: double.infinity,
      margin: const EdgeInsets.only(top: 8),
      padding: const EdgeInsets.symmetric(horizontal: 30),
      child: Wrap(
        alignment: WrapAlignment.spaceBetween,
        children: <Widget>[
          Row(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              ExcludeSemantics(
                child: SizedBox(
                  height: 80,
                  child: FadeInImagePlaceholder(
                    image: const AssetImage('logo.png', package: 'rally_assets'),
                    placeholder: LayoutBuilder(
                      builder: (BuildContext context, BoxConstraints constraints) {
                        return SizedBox(
                          width: constraints.maxHeight,
                          height: constraints.maxHeight,
                        );
                      },
                    ),
                  ),
                ),
              ),
              spacing,
              Text(
                localizations.rallyLoginLoginToRally,
                style: Theme.of(context).textTheme.bodyLarge!.copyWith(
                  fontSize: 35 / reducedTextScale(context),
                  fontWeight: FontWeight.w600,
                ),
              ),
            ],
          ),
          Row(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Text(
                localizations.rallyLoginNoAccount,
                style: Theme.of(context).textTheme.titleMedium,
              ),
              spacing,
              _BorderButton(text: localizations.rallyLoginSignUp),
            ],
          ),
        ],
      ),
    );
  }
}

class _SmallLogo extends StatelessWidget {
  const _SmallLogo();

  @override
  Widget build(BuildContext context) {
    return const Padding(
      padding: EdgeInsets.symmetric(vertical: 64),
      child: SizedBox(
        height: 160,
        child: ExcludeSemantics(
          child: FadeInImagePlaceholder(
            image: AssetImage('logo.png', package: 'rally_assets'),
            placeholder: SizedBox.shrink(),
          ),
        ),
      ),
    );
  }
}

class _UsernameInput extends StatelessWidget {
  const _UsernameInput({this.maxWidth, this.usernameController});

  final double? maxWidth;
  final TextEditingController? usernameController;

  @override
  Widget build(BuildContext context) {
    return Align(
      child: Container(
        constraints: BoxConstraints(maxWidth: maxWidth ?? double.infinity),
        child: TextField(
          autofillHints: const <String>[AutofillHints.username],
          textInputAction: TextInputAction.next,
          controller: usernameController,
          decoration: InputDecoration(
            labelText: GalleryLocalizations.of(context)!.rallyLoginUsername,
          ),
        ),
      ),
    );
  }
}

class _PasswordInput extends StatelessWidget {
  const _PasswordInput({this.maxWidth, this.passwordController});

  final double? maxWidth;
  final TextEditingController? passwordController;

  @override
  Widget build(BuildContext context) {
    return Align(
      child: Container(
        constraints: BoxConstraints(maxWidth: maxWidth ?? double.infinity),
        child: TextField(
          controller: passwordController,
          decoration: InputDecoration(
            labelText: GalleryLocalizations.of(context)!.rallyLoginPassword,
          ),
          obscureText: true,
        ),
      ),
    );
  }
}

class _ThumbButton extends StatefulWidget {
  const _ThumbButton({required this.onTap});

  final VoidCallback onTap;

  @override
  _ThumbButtonState createState() => _ThumbButtonState();
}

class _ThumbButtonState extends State<_ThumbButton> {
  BoxDecoration? borderDecoration;

  @override
  Widget build(BuildContext context) {
    return Semantics(
      button: true,
      enabled: true,
      label: GalleryLocalizations.of(context)!.rallyLoginLabelLogin,
      child: MouseRegion(
        cursor: SystemMouseCursors.click,
        child: GestureDetector(
          onTap: widget.onTap,
          child: Focus(
            onKeyEvent: (FocusNode node, KeyEvent event) {
              if (event is KeyDownEvent || event is KeyRepeatEvent) {
                if (event.logicalKey == LogicalKeyboardKey.enter ||
                    event.logicalKey == LogicalKeyboardKey.space) {
                  widget.onTap();
                  return KeyEventResult.handled;
                }
              }
              return KeyEventResult.ignored;
            },
            onFocusChange: (bool hasFocus) {
              if (hasFocus) {
                setState(() {
                  borderDecoration = BoxDecoration(
                    border: Border.all(color: Colors.white.withOpacity(0.5), width: 2),
                  );
                });
              } else {
                setState(() {
                  borderDecoration = null;
                });
              }
            },
            child: Container(
              decoration: borderDecoration,
              height: 120,
              child: ExcludeSemantics(child: Image.asset('thumb.png', package: 'rally_assets')),
            ),
          ),
        ),
      ),
    );
  }
}

class _LoginButton extends StatelessWidget {
  const _LoginButton({required this.onTap, this.maxWidth});

  final double? maxWidth;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return Align(
      child: Container(
        constraints: BoxConstraints(maxWidth: maxWidth ?? double.infinity),
        padding: const EdgeInsets.symmetric(vertical: 30),
        child: Row(
          children: <Widget>[
            const Icon(Icons.check_circle_outline, color: RallyColors.buttonColor),
            const SizedBox(width: 12),
            Text(GalleryLocalizations.of(context)!.rallyLoginRememberMe),
            const Expanded(child: SizedBox.shrink()),
            _FilledButton(
              text: GalleryLocalizations.of(context)!.rallyLoginButtonLogin,
              onTap: onTap,
            ),
          ],
        ),
      ),
    );
  }
}

class _BorderButton extends StatelessWidget {
  const _BorderButton({required this.text});

  final String text;

  @override
  Widget build(BuildContext context) {
    return OutlinedButton(
      style: OutlinedButton.styleFrom(
        foregroundColor: Colors.white,
        side: const BorderSide(color: RallyColors.buttonColor),
        padding: const EdgeInsets.symmetric(vertical: 14, horizontal: 24),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
      onPressed: () {
        Navigator.of(context).restorablePushNamed(RallyApp.homeRoute);
      },
      child: Text(text),
    );
  }
}

class _FilledButton extends StatelessWidget {
  const _FilledButton({required this.text, required this.onTap});

  final String text;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return TextButton(
      style: TextButton.styleFrom(
        foregroundColor: Colors.black,
        backgroundColor: RallyColors.buttonColor,
        padding: const EdgeInsets.symmetric(vertical: 10, horizontal: 24),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
      onPressed: onTap,
      child: Row(children: <Widget>[const Icon(Icons.lock), const SizedBox(width: 6), Text(text)]),
    );
  }
}
