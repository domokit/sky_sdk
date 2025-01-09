// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  test('Locale', () {
    expect(const Locale('en').toString(), 'en');
    expect(const Locale('en'), const Locale('en'));
    expect(const Locale('en').hashCode, const Locale('en').hashCode);
    expect(const Locale('en'), isNot(const Locale('en', '')));
    // TODO(het): This fails on web because "".hashCode == null.hashCode == 0
    //expect(const Locale('en').hashCode, isNot(const Locale('en', '').hashCode));
    expect(const Locale('en', 'US').toString(), 'en_US');
    expect(const Locale('iw').toString(), 'he');
    expect(const Locale('iw', 'DD').toString(), 'he_DE');
    expect(const Locale('iw', 'DD'), const Locale('he', 'DE'));
  });

  test('Locale.fromSubtags', () {
    expect(const Locale.fromSubtags().languageCode, 'und');
    expect(const Locale.fromSubtags().scriptCode, null);
    expect(const Locale.fromSubtags().countryCode, null);

    expect(const Locale.fromSubtags(languageCode: 'en').toString(), 'en');
    expect(const Locale.fromSubtags(languageCode: 'en').languageCode, 'en');
    expect(const Locale.fromSubtags(scriptCode: 'Latn').toString(), 'und_Latn');
    expect(const Locale.fromSubtags(scriptCode: 'Latn').scriptCode, 'Latn');
    expect(const Locale.fromSubtags(countryCode: 'US').toString(), 'und_US');
    expect(const Locale.fromSubtags(countryCode: 'US').countryCode, 'US');

    expect(const Locale.fromSubtags(languageCode: 'es', countryCode: '419').toString(), 'es_419');
    expect(const Locale.fromSubtags(languageCode: 'es', countryCode: '419').languageCode, 'es');
    expect(const Locale.fromSubtags(languageCode: 'es', countryCode: '419').countryCode, '419');

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
  });
}
