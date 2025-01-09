// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

const StandardMessageCodec codec = StandardMessageCodec();

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  late AccessibilityAnnouncements accessibilityAnnouncements;

  setUp(() {
    final DomElement announcementsHost = createDomElement('flt-announcement-host');
    accessibilityAnnouncements = AccessibilityAnnouncements(hostElement: announcementsHost);
    setLiveMessageDurationForTest(const Duration(milliseconds: 10));
    expect(
      announcementsHost.querySelector('flt-announcement-polite'),
      accessibilityAnnouncements.ariaLiveElementFor(Assertiveness.polite),
    );
    expect(
      announcementsHost.querySelector('flt-announcement-assertive'),
      accessibilityAnnouncements.ariaLiveElementFor(Assertiveness.assertive),
    );
  });

  tearDown(() async {
    await Future<void>.delayed(liveMessageDuration * 2);
  });

  group('$AccessibilityAnnouncements', () {
    ByteData? encodeMessageOnly({required String message}) {
      return codec.encodeMessage(<dynamic, dynamic>{
        'data': <dynamic, dynamic>{'message': message},
      });
    }

    void sendAnnouncementMessage({required String message, int? assertiveness}) {
      accessibilityAnnouncements.handleMessage(
        codec,
        codec.encodeMessage(<dynamic, dynamic>{
          'data': <dynamic, dynamic>{'message': message, 'assertiveness': assertiveness},
        }),
      );
    }

    void expectMessages({String polite = '', String assertive = ''}) {
      expect(accessibilityAnnouncements.ariaLiveElementFor(Assertiveness.polite).text, polite);
      expect(
        accessibilityAnnouncements.ariaLiveElementFor(Assertiveness.assertive).text,
        assertive,
      );
    }

    void expectNoMessages() => expectMessages();

    test('Default value of aria-live is polite when assertiveness is not specified', () async {
      accessibilityAnnouncements.handleMessage(codec, encodeMessageOnly(message: 'polite message'));
      expectMessages(polite: 'polite message');

      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();
    });

    test('aria-live is assertive when assertiveness is set to 1', () async {
      sendAnnouncementMessage(message: 'assertive message', assertiveness: 1);
      expectMessages(assertive: 'assertive message');

      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();
    });

    test('aria-live is polite when assertiveness is null', () async {
      sendAnnouncementMessage(message: 'polite message');
      expectMessages(polite: 'polite message');

      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();
    });

    test('aria-live is polite when assertiveness is set to 0', () async {
      sendAnnouncementMessage(message: 'polite message', assertiveness: 0);
      expectMessages(polite: 'polite message');

      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();
    });

    test('Rapid-fire messages are each announced', () async {
      sendAnnouncementMessage(message: 'Hello');
      expectMessages(polite: 'Hello');

      await Future<void>.delayed(liveMessageDuration * 0.5);
      sendAnnouncementMessage(message: 'There');
      expectMessages(polite: 'HelloThere\u00A0');

      await Future<void>.delayed(liveMessageDuration * 0.6);
      expectMessages(polite: 'There\u00A0');

      await Future<void>.delayed(liveMessageDuration * 0.5);
      expectNoMessages();
    });

    test('Repeated announcements are modified to ensure screen readers announce them', () async {
      sendAnnouncementMessage(message: 'Hello');
      expectMessages(polite: 'Hello');
      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();

      sendAnnouncementMessage(message: 'Hello');
      expectMessages(polite: 'Hello\u00A0');
      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();

      sendAnnouncementMessage(message: 'Hello');
      expectMessages(polite: 'Hello');
      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();
    });

    test('announce() polite', () async {
      accessibilityAnnouncements.announce('polite message', Assertiveness.polite);
      expectMessages(polite: 'polite message');

      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();
    });

    test('announce() assertive', () async {
      accessibilityAnnouncements.announce('assertive message', Assertiveness.assertive);
      expectMessages(assertive: 'assertive message');

      await Future<void>.delayed(liveMessageDuration);
      expectNoMessages();
    });
  });
}
