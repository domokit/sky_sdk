// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Deprecated(
  'This is the reason and what you should use instead. '
  'This feature was deprecated after v1.2.3.',
)
void test1() {}

// The code below is intentionally miss-formatted for testing.
// dart format off
@Deprecated(
  'bad grammar. '
  'This feature was deprecated after v1.2.3.'
)
void test2() { }

@Deprecated(
  'Also bad grammar '
  'This feature was deprecated after v1.2.3.'
)
void test3() { }

@Deprecated('Not the right syntax. This feature was deprecated after v1.2.3.')
void test4() { }

@Deprecated(
  'Missing the version line. '
)
void test5() { }

@Deprecated(
  'This feature was deprecated after v1.2.3.'
)
void test6() { }

@Deprecated(
  'URLs are not required. '
  'This feature was deprecated after v1.0.0.'
)
void test7() { }

@Deprecated(
  'Version number test (should pass). '
  'This feature was deprecated after v1.19.0.'
)
void test8() { }

@Deprecated(
  'Version number test (should fail). '
  'This feature was deprecated after v1.20.0.'
)
void test9() { }

@Deprecated(
  'Version number test (should fail). '
  'This feature was deprecated after v1.21.0.'
)
void test10() { }

@Deprecated(
  'Version number test (special beta should pass). '
  'This feature was deprecated after v3.1.0.'
)
void test11() { }

@Deprecated(
  'Version number test (should be fine). '
  'This feature was deprecated after v0.1.0.'
)
void test12() { }

@Deprecated(
  'Version number test (should be fine). '
  'This feature was deprecated after v1.20.0-1.0.pre.'
)
void test13() { }

@Deprecated(
  "Double quotes' test (should fail). "
  'This feature was deprecated after v2.1.0-11.0.pre.'
)
void test14() { }

@Deprecated( // flutter_ignore: deprecation_syntax, https://github.com/flutter/flutter/issues/000000
  'Missing the version line. '
)
void test15() { }
// dart format on

// flutter_ignore: deprecation_syntax, https://github.com/flutter/flutter/issues/000000
@Deprecated('Missing the version line. ')
void test16() {}

class TestClass1 {
  // flutter_ignore: deprecation_syntax, https://github.com/flutter/flutter/issues/000000
  @Deprecated('Missing the version line. ')
  void test() {}
}
