// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('Locale', () {
    expect(const Locale('en').toLanguageTag(), 'en');
    expect(const Locale('en'), const Locale('en'));
    expect(const Locale('en').hashCode, const Locale('en').hashCode);
    expect(const Locale('en', 'US').toLanguageTag(), 'en-US');
    expect(const Locale('en', 'US').toString(), 'en_US');
    expect(const Locale('iw').toLanguageTag(), 'he');
    expect(const Locale('iw', 'DD').toLanguageTag(), 'he-DE');
    expect(const Locale('iw', 'DD').toString(), 'he_DE');
    expect(const Locale('iw', 'DD'), const Locale('he', 'DE'));
  });

  test('Locale.fromSubtags', () {
    expect(const Locale.fromSubtags().languageCode, 'und');
    expect(const Locale.fromSubtags().scriptCode, null);
    expect(const Locale.fromSubtags().countryCode, null);

    expect(const Locale.fromSubtags(languageCode: 'en').toLanguageTag(), 'en');
    expect(const Locale.fromSubtags(languageCode: 'en').languageCode, 'en');
    expect(const Locale.fromSubtags(scriptCode: 'Latn').toLanguageTag(), 'und-Latn');
    expect(const Locale.fromSubtags(scriptCode: 'Latn').toString(), 'und_Latn');
    expect(const Locale.fromSubtags(scriptCode: 'Latn').scriptCode, 'Latn');
    expect(const Locale.fromSubtags(countryCode: 'US').toLanguageTag(), 'und-US');
    expect(const Locale.fromSubtags(countryCode: 'US').toString(), 'und_US');
    expect(const Locale.fromSubtags(countryCode: 'US').countryCode, 'US');

    expect(
      const Locale.fromSubtags(languageCode: 'es', countryCode: '419').toLanguageTag(),
      'es-419',
    );
    expect(const Locale.fromSubtags(languageCode: 'es', countryCode: '419').toString(), 'es_419');
    expect(const Locale.fromSubtags(languageCode: 'es', countryCode: '419').languageCode, 'es');
    expect(const Locale.fromSubtags(languageCode: 'es', countryCode: '419').scriptCode, null);
    expect(const Locale.fromSubtags(languageCode: 'es', countryCode: '419').countryCode, '419');

    expect(
      const Locale.fromSubtags(
        languageCode: 'zh',
        scriptCode: 'Hans',
        countryCode: 'CN',
      ).toLanguageTag(),
      'zh-Hans-CN',
    );
    expect(
      const Locale.fromSubtags(
        languageCode: 'zh',
        scriptCode: 'Hans',
        countryCode: 'CN',
      ).toString(),
      'zh_Hans_CN',
    );
  });

  test('Locale equality', () {
    expect(
      const Locale.fromSubtags(languageCode: 'en'),
      isNot(const Locale.fromSubtags(languageCode: 'en', scriptCode: 'Latn')),
    );
    expect(
      const Locale.fromSubtags(languageCode: 'en').hashCode,
      isNot(const Locale.fromSubtags(languageCode: 'en', scriptCode: 'Latn').hashCode),
    );

    expect(const Locale('en', ''), const Locale('en'));
    expect(const Locale('en'), const Locale('en', ''));
    expect(const Locale('en'), const Locale('en'));
    expect(const Locale('en', ''), const Locale('en', ''));

    expect(const Locale('en', ''), isNot(const Locale('en', 'GB')));
    expect(const Locale('en'), isNot(const Locale('en', 'GB')));
    expect(const Locale('en', 'GB'), isNot(const Locale('en', '')));
    expect(const Locale('en', 'GB'), isNot(const Locale('en')));
  });

  test("Locale toString does not include separator for ''", () {
    expect(const Locale('en').toString(), 'en');
    expect(const Locale('en', '').toString(), 'en');
    expect(const Locale('en', 'US').toString(), 'en_US');
  });
}
