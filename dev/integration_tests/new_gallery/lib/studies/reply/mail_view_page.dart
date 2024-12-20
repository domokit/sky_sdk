// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'model/email_model.dart';
import 'model/email_store.dart';
import 'profile_avatar.dart';

class MailViewPage extends StatelessWidget {
  const MailViewPage({super.key, required this.id, required this.email});

  final int id;
  final Email email;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        bottom: false,
        child: SizedBox(
          height: double.infinity,
          child: Material(
            color: Theme.of(context).cardColor,
            child: SingleChildScrollView(
              padding: const EdgeInsetsDirectional.only(top: 42, start: 20, end: 20),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  _MailViewHeader(email: email),
                  const SizedBox(height: 32),
                  _MailViewBody(message: email.message),
                  if (email.containsPictures) ...<Widget>[
                    const SizedBox(height: 28),
                    const _PictureGrid(),
                  ],
                  const SizedBox(height: kToolbarHeight),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

class _MailViewHeader extends StatelessWidget {
  const _MailViewHeader({required this.email});

  final Email email;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;

    return Column(
      children: <Widget>[
        Row(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            Expanded(
              child: SelectableText(
                email.subject,
                style: textTheme.headlineMedium!.copyWith(height: 1.1),
              ),
            ),
            IconButton(
              key: const ValueKey<String>('ReplyExit'),
              icon: const Icon(Icons.keyboard_arrow_down),
              onPressed: () {
                Provider.of<EmailStore>(context, listen: false).selectedEmailId = -1;
                Navigator.pop(context);
              },
              splashRadius: 20,
            ),
          ],
        ),
        const SizedBox(height: 16),
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: <Widget>[
                SelectableText('${email.sender} - ${email.time}'),
                const SizedBox(height: 4),
                SelectableText(
                  'To ${email.recipients},',
                  style: textTheme.bodySmall!.copyWith(
                    color: Theme.of(context).navigationRailTheme.unselectedLabelTextStyle!.color,
                  ),
                ),
              ],
            ),
            Padding(
              padding: const EdgeInsetsDirectional.only(end: 4),
              child: ProfileAvatar(avatar: email.avatar),
            ),
          ],
        ),
      ],
    );
  }
}

class _MailViewBody extends StatelessWidget {
  const _MailViewBody({required this.message});

  final String message;

  @override
  Widget build(BuildContext context) {
    return SelectableText(
      message,
      style: Theme.of(context).textTheme.bodyMedium!.copyWith(fontSize: 16),
    );
  }
}

class _PictureGrid extends StatelessWidget {
  const _PictureGrid();

  bool _shouldShrinkImage() {
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.android:
        return true;
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        return false;
    }
  }

  @override
  Widget build(BuildContext context) {
    return GridView.builder(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
        crossAxisCount: 2,
        crossAxisSpacing: 4,
        mainAxisSpacing: 4,
      ),
      itemCount: 4,
      itemBuilder: (BuildContext context, int index) {
        return Image.asset(
          'reply/attachments/paris_${index + 1}.jpg',
          gaplessPlayback: true,
          package: 'flutter_gallery_assets',
          fit: BoxFit.fill,
          cacheWidth: _shouldShrinkImage() ? 500 : null,
        );
      },
    );
  }
}
