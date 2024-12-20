//---------------------------------------------------------------------------------------------
//  Copyright (c) 2022 Google LLC
//  Licensed under the MIT License. See License.txt in the project root for license information.
//--------------------------------------------------------------------------------------------*/


import 'package:test/test.dart';
import 'package:web_locale_keymap/web_locale_keymap.dart';

import 'test_cases.g.dart';

void main() {
  group('Win', () {
    testWin(LocaleKeymap.win());
  });

  group('Linux', () {
    testLinux(LocaleKeymap.linux());
  });

  group('Darwin', () {
    testDarwin(LocaleKeymap.darwin());
  });
}
