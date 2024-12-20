// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../src/common.dart';

void main() {
  group('containsIgnoreWhitespace Matcher', () {
    group('on item to be contained', () {
      test('matches simple case.', () {
        expect('Give me any text!', containsIgnoringWhitespace('any text!'));
      });

      test("shouldn't match when it's only because of removing whitespaces", () {
        expect('Give me anytext!', isNot(containsIgnoringWhitespace('any text!')));
      });

      test('ignores trailing spaces.', () {
        expect('Give me any text!', containsIgnoringWhitespace('any text!    '));
      });

      test('ignores leading spaces.', () {
        expect('Give me any text!', containsIgnoringWhitespace('   any text!'));
      });

      test('ignores linebreaks.', () {
        expect('Give me any text!', containsIgnoringWhitespace('any\n text!'));
      });

      test('ignores tabs.', () {
        expect('Give me any text!', containsIgnoringWhitespace('any\t text!'));
      });

      test('is case sensitive.', () {
        expect('Give me Any text!', isNot(containsIgnoringWhitespace('any text!')));
      });
    });

    group('on value to match against', () {
      test('ignores trailing spaces.', () {
        expect(
          'Give  me  any value to include!   ',
          containsIgnoringWhitespace('any value to include!'),
        );
      });

      test('ignores leading spaces.', () {
        expect(
          '     Give me    any value to include!',
          containsIgnoringWhitespace('any value to include!'),
        );
      });

      test('ignores linebreaks.', () {
        expect(
          'Give me \n any \n value \n to \n include!',
          containsIgnoringWhitespace('any value to include!'),
        );
      });

      test('ignores tabs.', () {
        expect(
          '\tGive\t me any\t value \t to \t include!',
          containsIgnoringWhitespace('any value to include!'),
        );
      });
    });
  });
}
