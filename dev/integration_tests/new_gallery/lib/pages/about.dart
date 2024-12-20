// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:url_launcher/url_launcher.dart';

import '../gallery_localizations.dart';

void showAboutDialog({required BuildContext context}) {
  showDialog<void>(
    context: context,
    builder: (BuildContext context) {
      return _AboutDialog();
    },
  );
}

Future<String> getVersionNumber() async {
  return '2.10.2+021002';
}

class _AboutDialog extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final TextTheme textTheme = Theme.of(context).textTheme;
    final TextStyle bodyTextStyle = textTheme.bodyLarge!.apply(color: colorScheme.onPrimary);
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    const String name = 'Flutter Gallery'; // Don't need to localize.
    const String legalese = '© 2021 The Flutter team'; // Don't need to localize.
    final String repoText = localizations.githubRepo(name);
    final String seeSource = localizations.aboutDialogDescription(repoText);
    final int repoLinkIndex = seeSource.indexOf(repoText);
    final int repoLinkIndexEnd = repoLinkIndex + repoText.length;
    final String seeSourceFirst = seeSource.substring(0, repoLinkIndex);
    final String seeSourceSecond = seeSource.substring(repoLinkIndexEnd);

    return AlertDialog(
      backgroundColor: colorScheme.background,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
      content: Container(
        constraints: const BoxConstraints(maxWidth: 400),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            FutureBuilder<String>(
              future: getVersionNumber(),
              builder:
                  (BuildContext context, AsyncSnapshot<String> snapshot) => SelectableText(
                    snapshot.hasData ? '$name ${snapshot.data}' : name,
                    style: textTheme.headlineMedium!.apply(color: colorScheme.onPrimary),
                  ),
            ),
            const SizedBox(height: 24),
            SelectableText.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(style: bodyTextStyle, text: seeSourceFirst),
                  TextSpan(
                    style: bodyTextStyle.copyWith(color: colorScheme.primary),
                    text: repoText,
                    recognizer:
                        TapGestureRecognizer()
                          ..onTap = () async {
                            final Uri url = Uri.parse('https://github.com/flutter/gallery/');
                            if (await canLaunchUrl(url)) {
                              await launchUrl(url);
                            }
                          },
                  ),
                  TextSpan(style: bodyTextStyle, text: seeSourceSecond),
                ],
              ),
            ),
            const SizedBox(height: 18),
            SelectableText(legalese, style: bodyTextStyle),
          ],
        ),
      ),
      actions: <Widget>[
        TextButton(
          onPressed: () {
            Navigator.of(context).push(
              MaterialPageRoute<void>(
                builder:
                    (BuildContext context) => Theme(
                      data: Theme.of(context).copyWith(
                        textTheme:
                            Typography.material2018(platform: Theme.of(context).platform).black,
                        cardColor: Colors.white,
                      ),
                      child: const LicensePage(
                        applicationName: name,
                        applicationLegalese: legalese,
                      ),
                    ),
              ),
            );
          },
          child: Text(MaterialLocalizations.of(context).viewLicensesButtonLabel),
        ),
        TextButton(
          onPressed: () {
            Navigator.pop(context);
          },
          child: Text(MaterialLocalizations.of(context).closeButtonLabel),
        ),
      ],
    );
  }
}
