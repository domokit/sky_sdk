// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../../layout/adaptive.dart';
import 'mail_card_preview.dart';
import 'model/email_model.dart';
import 'model/email_store.dart';

class MailboxBody extends StatelessWidget {
  const MailboxBody({super.key});

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final bool isTablet = isDisplaySmallDesktop(context);
    final double startPadding =
        isTablet
            ? 60.0
            : isDesktop
            ? 120.0
            : 4.0;
    final double endPadding =
        isTablet
            ? 30.0
            : isDesktop
            ? 60.0
            : 4.0;

    return Consumer<EmailStore>(
      builder: (BuildContext context, EmailStore model, Widget? child) {
        final MailboxPageType destination = model.selectedMailboxPage;
        final String destinationString = destination.toString().substring(
          destination.toString().indexOf('.') + 1,
        );

        final List<Email> emails = switch (destination) {
          MailboxPageType.inbox => model.inboxEmails,
          MailboxPageType.sent => model.outboxEmails,
          MailboxPageType.starred => model.starredEmails,
          MailboxPageType.trash => model.trashEmails,
          MailboxPageType.spam => model.spamEmails,
          MailboxPageType.drafts => model.draftEmails,
        };

        return SafeArea(
          bottom: false,
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              Expanded(
                child:
                    emails.isEmpty
                        ? Center(child: Text('Empty in $destinationString'))
                        : ListView.separated(
                          itemCount: emails.length,
                          padding: EdgeInsetsDirectional.only(
                            start: startPadding,
                            end: endPadding,
                            top: isDesktop ? 28 : 0,
                            bottom: kToolbarHeight,
                          ),
                          primary: false,
                          separatorBuilder:
                              (BuildContext context, int index) => const SizedBox(height: 4),
                          itemBuilder: (BuildContext context, int index) {
                            final Email email = emails[index];
                            return MailPreviewCard(
                              id: email.id,
                              email: email,
                              isStarred: model.isEmailStarred(email.id),
                              onDelete: () => model.deleteEmail(email.id),
                              onStar: () {
                                final int emailId = email.id;
                                if (model.isEmailStarred(emailId)) {
                                  model.unstarEmail(emailId);
                                } else {
                                  model.starEmail(emailId);
                                }
                              },
                              onStarredMailbox:
                                  model.selectedMailboxPage == MailboxPageType.starred,
                            );
                          },
                        ),
              ),
              if (isDesktop) ...<Widget>[
                Padding(
                  padding: const EdgeInsetsDirectional.only(top: 14),
                  child: Row(
                    children: <Widget>[
                      IconButton(
                        key: const ValueKey<String>('ReplySearch'),
                        icon: const Icon(Icons.search),
                        onPressed: () {
                          Provider.of<EmailStore>(context, listen: false).onSearchPage = true;
                        },
                      ),
                      SizedBox(width: isTablet ? 30 : 60),
                    ],
                  ),
                ),
              ],
            ],
          ),
        );
      },
    );
  }
}
