// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

class Email {
  Email({
    required this.id,
    required this.avatar,
    this.sender = '',
    this.time = '',
    this.subject = '',
    this.message = '',
    this.recipients = '',
    this.containsPictures = false,
  });

  final int id;
  final String sender;
  final String time;
  final String subject;
  final String message;
  final String avatar;
  final String recipients;
  final bool containsPictures;
}

class InboxEmail extends Email {
  InboxEmail({
    required super.id,
    required super.sender,
    super.time,
    super.subject,
    super.message,
    required super.avatar,
    super.recipients,
    super.containsPictures,
    this.inboxType = InboxType.normal,
  });

  InboxType inboxType;
}

// The different mailbox pages that the Reply app contains.
enum MailboxPageType { inbox, starred, sent, trash, spam, drafts }

// Different types of mail that can be sent to the inbox.
enum InboxType { normal, spam }
