// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:animations/animations.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../../layout/adaptive.dart';
import 'colors.dart';
import 'mail_view_page.dart';
import 'model/email_model.dart';
import 'model/email_store.dart';
import 'profile_avatar.dart';

const String _assetsPackage = 'flutter_gallery_assets';
const String _iconAssetLocation = 'reply/icons';

class MailPreviewCard extends StatelessWidget {
  const MailPreviewCard({
    super.key,
    required this.id,
    required this.email,
    required this.onDelete,
    required this.onStar,
    required this.isStarred,
    required this.onStarredMailbox,
  });

  final int id;
  final Email email;
  final VoidCallback onDelete;
  final VoidCallback onStar;
  final bool isStarred;
  final bool onStarredMailbox;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);

    // TODO(x): State restoration of mail view page is blocked because OpenContainer does not support restorablePush, https://github.com/flutter/gallery/issues/570.
    return OpenContainer(
      openBuilder: (BuildContext context, void Function() closedContainer) {
        return MailViewPage(id: id, email: email);
      },
      openColor: theme.cardColor,
      closedShape: const RoundedRectangleBorder(),
      closedElevation: 0,
      closedColor: theme.cardColor,
      closedBuilder: (BuildContext context, void Function() openContainer) {
        final bool isDesktop = isDisplayDesktop(context);
        final ColorScheme colorScheme = theme.colorScheme;
        final _MailPreview mailPreview = _MailPreview(
          id: id,
          email: email,
          onTap: openContainer,
          onStar: onStar,
          onDelete: onDelete,
        );

        if (isDesktop) {
          return mailPreview;
        } else {
          return Dismissible(
            key: ObjectKey(email),
            dismissThresholds: const <DismissDirection, double>{
              DismissDirection.startToEnd: 0.8,
              DismissDirection.endToStart: 0.4,
            },
            onDismissed: (DismissDirection direction) {
              switch (direction) {
                case DismissDirection.endToStart:
                  if (onStarredMailbox) {
                    onStar();
                  }
                case DismissDirection.startToEnd:
                  onDelete();
                case DismissDirection.vertical:
                case DismissDirection.horizontal:
                case DismissDirection.up:
                case DismissDirection.down:
                case DismissDirection.none:
                  break;
              }
            },
            background: _DismissibleContainer(
              icon: 'twotone_delete',
              backgroundColor: colorScheme.primary,
              iconColor: ReplyColors.blue50,
              alignment: Alignment.centerLeft,
              padding: const EdgeInsetsDirectional.only(start: 20),
            ),
            confirmDismiss: (DismissDirection direction) async {
              if (direction == DismissDirection.endToStart) {
                if (onStarredMailbox) {
                  return true;
                }
                onStar();
                return false;
              } else {
                return true;
              }
            },
            secondaryBackground: _DismissibleContainer(
              icon: 'twotone_star',
              backgroundColor: isStarred ? colorScheme.secondary : theme.scaffoldBackgroundColor,
              iconColor: isStarred ? colorScheme.onSecondary : colorScheme.onBackground,
              alignment: Alignment.centerRight,
              padding: const EdgeInsetsDirectional.only(end: 20),
            ),
            child: mailPreview,
          );
        }
      },
    );
  }
}

class _DismissibleContainer extends StatelessWidget {
  const _DismissibleContainer({
    required this.icon,
    required this.backgroundColor,
    required this.iconColor,
    required this.alignment,
    required this.padding,
  });

  final String icon;
  final Color backgroundColor;
  final Color iconColor;
  final Alignment alignment;
  final EdgeInsetsDirectional padding;

  @override
  Widget build(BuildContext context) {
    return AnimatedContainer(
      alignment: alignment,
      curve: Easing.legacy,
      color: backgroundColor,
      duration: kThemeAnimationDuration,
      padding: padding,
      child: Material(
        color: Colors.transparent,
        child: ImageIcon(
          AssetImage('reply/icons/$icon.png', package: 'flutter_gallery_assets'),
          size: 36,
          color: iconColor,
        ),
      ),
    );
  }
}

class _MailPreview extends StatelessWidget {
  const _MailPreview({
    required this.id,
    required this.email,
    required this.onTap,
    this.onStar,
    this.onDelete,
  });

  final int id;
  final Email email;
  final VoidCallback onTap;
  final VoidCallback? onStar;
  final VoidCallback? onDelete;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final EmailStore emailStore = Provider.of<EmailStore>(context, listen: false);

    return InkWell(
      onTap: () {
        Provider.of<EmailStore>(context, listen: false).selectedEmailId = id;
        onTap();
      },
      child: LayoutBuilder(
        builder: (BuildContext context, BoxConstraints constraints) {
          return ConstrainedBox(
            constraints: BoxConstraints(maxHeight: constraints.maxHeight),
            child: Padding(
              padding: const EdgeInsets.all(20),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                mainAxisSize: MainAxisSize.min,
                children: <Widget>[
                  Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                          children: <Widget>[
                            Text('${email.sender} - ${email.time}', style: textTheme.bodySmall),
                            const SizedBox(height: 4),
                            Text(email.subject, style: textTheme.headlineSmall),
                            const SizedBox(height: 16),
                          ],
                        ),
                      ),
                      _MailPreviewActionBar(
                        avatar: email.avatar,
                        isStarred: emailStore.isEmailStarred(email.id),
                        onStar: onStar,
                        onDelete: onDelete,
                      ),
                    ],
                  ),
                  Padding(
                    padding: const EdgeInsetsDirectional.only(end: 20),
                    child: Text(
                      email.message,
                      overflow: TextOverflow.ellipsis,
                      maxLines: 1,
                      style: textTheme.bodyMedium,
                    ),
                  ),
                  if (email.containsPictures) ...<Widget>[
                    const Flexible(
                      child: Column(children: <Widget>[SizedBox(height: 20), _PicturePreview()]),
                    ),
                  ],
                ],
              ),
            ),
          );
        },
      ),
    );
  }
}

class _PicturePreview extends StatelessWidget {
  const _PicturePreview();

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
    return SizedBox(
      height: 96,
      child: ListView.builder(
        itemCount: 4,
        scrollDirection: Axis.horizontal,
        itemBuilder: (BuildContext context, int index) {
          return Padding(
            padding: const EdgeInsetsDirectional.only(end: 4),
            child: Image.asset(
              'reply/attachments/paris_${index + 1}.jpg',
              gaplessPlayback: true,
              package: 'flutter_gallery_assets',
              cacheWidth: _shouldShrinkImage() ? 200 : null,
            ),
          );
        },
      ),
    );
  }
}

class _MailPreviewActionBar extends StatelessWidget {
  const _MailPreviewActionBar({
    required this.avatar,
    required this.isStarred,
    this.onStar,
    this.onDelete,
  });

  final String avatar;
  final bool isStarred;
  final VoidCallback? onStar;
  final VoidCallback? onDelete;

  @override
  Widget build(BuildContext context) {
    final bool isDark = Theme.brightnessOf(context) == Brightness.dark;
    final Color color = isDark ? ReplyColors.white50 : ReplyColors.blue600;
    final bool isDesktop = isDisplayDesktop(context);
    final Color starredIconColor = isStarred ? Theme.of(context).colorScheme.secondary : color;

    return Row(
      children: <Widget>[
        if (isDesktop) ...<Widget>[
          IconButton(
            icon: ImageIcon(
              const AssetImage('$_iconAssetLocation/twotone_star.png', package: _assetsPackage),
              color: starredIconColor,
            ),
            onPressed: onStar,
          ),
          IconButton(
            icon: ImageIcon(
              const AssetImage('$_iconAssetLocation/twotone_delete.png', package: _assetsPackage),
              color: color,
            ),
            onPressed: onDelete,
          ),
          IconButton(icon: Icon(Icons.more_vert, color: color), onPressed: () {}),
          const SizedBox(width: 12),
        ],
        ProfileAvatar(avatar: avatar),
      ],
    );
  }
}
