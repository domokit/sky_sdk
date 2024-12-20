// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/localizations/gen_l10n_types.dart';
import 'package:flutter_tools/src/localizations/message_parser.dart';
import '../src/common.dart';

void main() {
  // Going to test that operator== is overloaded properly since the rest
  // of the test depends on it.
  testWithoutContext('node equality', () {
    final Node actual = Node(
      ST.placeholderExpr,
      0,
      expectedSymbolCount: 3,
      children: <Node>[Node.openBrace(0), Node.string(1, 'var'), Node.closeBrace(4)],
    );

    final Node expected = Node(
      ST.placeholderExpr,
      0,
      expectedSymbolCount: 3,
      children: <Node>[Node.openBrace(0), Node.string(1, 'var'), Node.closeBrace(4)],
    );
    expect(actual, equals(expected));

    final Node wrongType = Node(
      ST.pluralExpr,
      0,
      expectedSymbolCount: 3,
      children: <Node>[Node.openBrace(0), Node.string(1, 'var'), Node.closeBrace(4)],
    );
    expect(actual, isNot(equals(wrongType)));

    final Node wrongPosition = Node(
      ST.placeholderExpr,
      1,
      expectedSymbolCount: 3,
      children: <Node>[Node.openBrace(0), Node.string(1, 'var'), Node.closeBrace(4)],
    );
    expect(actual, isNot(equals(wrongPosition)));

    final Node wrongChildrenCount = Node(
      ST.placeholderExpr,
      0,
      expectedSymbolCount: 3,
      children: <Node>[Node.string(1, 'var'), Node.closeBrace(4)],
    );
    expect(actual, isNot(equals(wrongChildrenCount)));

    final Node wrongChild = Node(
      ST.placeholderExpr,
      0,
      expectedSymbolCount: 3,
      children: <Node>[Node.closeBrace(0), Node.string(1, 'var'), Node.closeBrace(4)],
    );
    expect(actual, isNot(equals(wrongChild)));
  });

  testWithoutContext('lexer basic', () {
    final List<Node> tokens1 = Parser('helloWorld', 'app_en.arb', 'Hello {name}').lexIntoTokens();
    expect(
      tokens1,
      equals(<Node>[
        Node.string(0, 'Hello '),
        Node.openBrace(6),
        Node.identifier(7, 'name'),
        Node.closeBrace(11),
      ]),
    );

    final List<Node> tokens2 =
        Parser(
          'plural',
          'app_en.arb',
          'There are {count} {count, plural, =1{cat} other{cats}}',
        ).lexIntoTokens();
    expect(
      tokens2,
      equals(<Node>[
        Node.string(0, 'There are '),
        Node.openBrace(10),
        Node.identifier(11, 'count'),
        Node.closeBrace(16),
        Node.string(17, ' '),
        Node.openBrace(18),
        Node.identifier(19, 'count'),
        Node.comma(24),
        Node.pluralKeyword(26),
        Node.comma(32),
        Node.equalSign(34),
        Node.number(35, '1'),
        Node.openBrace(36),
        Node.string(37, 'cat'),
        Node.closeBrace(40),
        Node.otherKeyword(42),
        Node.openBrace(47),
        Node.string(48, 'cats'),
        Node.closeBrace(52),
        Node.closeBrace(53),
      ]),
    );

    final List<Node> tokens3 =
        Parser(
          'gender',
          'app_en.arb',
          '{gender, select, male{he} female{she} other{they}}',
        ).lexIntoTokens();
    expect(
      tokens3,
      equals(<Node>[
        Node.openBrace(0),
        Node.identifier(1, 'gender'),
        Node.comma(7),
        Node.selectKeyword(9),
        Node.comma(15),
        Node.identifier(17, 'male'),
        Node.openBrace(21),
        Node.string(22, 'he'),
        Node.closeBrace(24),
        Node.identifier(26, 'female'),
        Node.openBrace(32),
        Node.string(33, 'she'),
        Node.closeBrace(36),
        Node.otherKeyword(38),
        Node.openBrace(43),
        Node.string(44, 'they'),
        Node.closeBrace(48),
        Node.closeBrace(49),
      ]),
    );
  });

  testWithoutContext('lexer recursive', () {
    final List<Node> tokens =
        Parser(
          'plural',
          'app_en.arb',
          '{count, plural, =1{{gender, select, male{he} female{she}}} other{they}}',
        ).lexIntoTokens();
    expect(
      tokens,
      equals(<Node>[
        Node.openBrace(0),
        Node.identifier(1, 'count'),
        Node.comma(6),
        Node.pluralKeyword(8),
        Node.comma(14),
        Node.equalSign(16),
        Node.number(17, '1'),
        Node.openBrace(18),
        Node.openBrace(19),
        Node.identifier(20, 'gender'),
        Node.comma(26),
        Node.selectKeyword(28),
        Node.comma(34),
        Node.identifier(36, 'male'),
        Node.openBrace(40),
        Node.string(41, 'he'),
        Node.closeBrace(43),
        Node.identifier(45, 'female'),
        Node.openBrace(51),
        Node.string(52, 'she'),
        Node.closeBrace(55),
        Node.closeBrace(56),
        Node.closeBrace(57),
        Node.otherKeyword(59),
        Node.openBrace(64),
        Node.string(65, 'they'),
        Node.closeBrace(69),
        Node.closeBrace(70),
      ]),
    );
  });

  testWithoutContext('lexer escaping', () {
    final List<Node> tokens1 =
        Parser('escaping', 'app_en.arb', "''", useEscaping: true).lexIntoTokens();
    expect(tokens1, equals(<Node>[Node.string(0, "'")]));

    final List<Node> tokens2 =
        Parser(
          'escaping',
          'app_en.arb',
          "'hello world { name }'",
          useEscaping: true,
        ).lexIntoTokens();
    expect(tokens2, equals(<Node>[Node.string(0, 'hello world { name }')]));

    final List<Node> tokens3 =
        Parser(
          'escaping',
          'app_en.arb',
          "'{ escaped string }' { not escaped }",
          useEscaping: true,
        ).lexIntoTokens();
    expect(
      tokens3,
      equals(<Node>[
        Node.string(0, '{ escaped string }'),
        Node.string(20, ' '),
        Node.openBrace(21),
        Node.identifier(23, 'not'),
        Node.identifier(27, 'escaped'),
        Node.closeBrace(35),
      ]),
    );

    final List<Node> tokens4 =
        Parser('escaping', 'app_en.arb', "Flutter''s amazing!", useEscaping: true).lexIntoTokens();
    expect(
      tokens4,
      equals(<Node>[Node.string(0, 'Flutter'), Node.string(7, "'"), Node.string(9, 's amazing!')]),
    );

    final List<Node> tokens5 =
        Parser(
          'escaping',
          'app_en.arb',
          "'Flutter''s amazing!'",
          useEscaping: true,
        ).lexIntoTokens();
    expect(
      tokens5,
      equals(<Node>[
        Node(ST.string, 0, value: 'Flutter'),
        Node(ST.string, 9, value: "'s amazing!"),
      ]),
    );
  });

  testWithoutContext('lexer identifier names can be "select" or "plural"', () {
    final List<Node> tokens =
        Parser(
          'keywords',
          'app_en.arb',
          '{ select } { plural, select, singular{test} other{hmm} }',
        ).lexIntoTokens();
    expect(tokens[1].value, equals('select'));
    expect(tokens[1].type, equals(ST.identifier));
    expect(tokens[5].value, equals('plural'));
    expect(tokens[5].type, equals(ST.identifier));
  });

  testWithoutContext('lexer identifier names can contain underscores', () {
    final List<Node> tokens =
        Parser(
          'keywords',
          'app_en.arb',
          '{ test_placeholder } { test_select, select, singular{test} other{hmm} }',
        ).lexIntoTokens();
    expect(tokens[1].value, equals('test_placeholder'));
    expect(tokens[1].type, equals(ST.identifier));
    expect(tokens[5].value, equals('test_select'));
    expect(tokens[5].type, equals(ST.identifier));
  });

  testWithoutContext('lexer identifier names can contain the strings select or plural', () {
    final List<Node> tokens =
        Parser(
          'keywords',
          'app_en.arb',
          '{ selectTest } { pluralTest, select, singular{test} other{hmm} }',
        ).lexIntoTokens();
    expect(tokens[1].value, equals('selectTest'));
    expect(tokens[1].type, equals(ST.identifier));
    expect(tokens[5].value, equals('pluralTest'));
    expect(tokens[5].type, equals(ST.identifier));
  });

  testWithoutContext('lexer: lexically correct but syntactically incorrect', () {
    final List<Node> tokens =
        Parser(
          'syntax',
          'app_en.arb',
          'string { identifier { string { identifier } } }',
        ).lexIntoTokens();
    expect(
      tokens,
      equals(<Node>[
        Node.string(0, 'string '),
        Node.openBrace(7),
        Node.identifier(9, 'identifier'),
        Node.openBrace(20),
        Node.string(21, ' string '),
        Node.openBrace(29),
        Node.identifier(31, 'identifier'),
        Node.closeBrace(42),
        Node.string(43, ' '),
        Node.closeBrace(44),
        Node.closeBrace(46),
      ]),
    );
  });

  testWithoutContext('lexer unmatched single quote', () {
    const String message = "here''s an unmatched single quote: '";
    const String expectedError = '''
[app_en.arb:escaping] ICU Lexing Error: Unmatched single quotes.
    here''s an unmatched single quote: '
                                       ^''';
    expect(
      () => Parser('escaping', 'app_en.arb', message, useEscaping: true).lexIntoTokens(),
      throwsA(
        isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains(expectedError),
        ),
      ),
    );
  });

  testWithoutContext('lexer unexpected character', () {
    const String message = '{ * }';
    const String expectedError = '''
[app_en.arb:lex] ICU Lexing Error: Unexpected character.
    { * }
      ^''';
    expect(
      () => Parser('lex', 'app_en.arb', message).lexIntoTokens(),
      throwsA(
        isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains(expectedError),
        ),
      ),
    );
  });

  testWithoutContext('relaxed lexer', () {
    final List<Node> tokens1 =
        Parser('string', 'app_en.arb', '{ }', placeholders: <String>[]).lexIntoTokens();
    expect(
      tokens1,
      equals(<Node>[
        Node(ST.string, 0, value: '{'),
        Node(ST.string, 1, value: ' '),
        Node(ST.string, 2, value: '}'),
      ]),
    );

    final List<Node> tokens2 =
        Parser(
          'string',
          'app_en.arb',
          '{ notAPlaceholder }',
          placeholders: <String>['isAPlaceholder'],
        ).lexIntoTokens();
    expect(
      tokens2,
      equals(<Node>[
        Node(ST.string, 0, value: '{'),
        Node(ST.string, 1, value: ' notAPlaceholder '),
        Node(ST.string, 18, value: '}'),
      ]),
    );

    final List<Node> tokens3 =
        Parser(
          'string',
          'app_en.arb',
          '{ isAPlaceholder }',
          placeholders: <String>['isAPlaceholder'],
        ).lexIntoTokens();
    expect(
      tokens3,
      equals(<Node>[
        Node(ST.openBrace, 0, value: '{'),
        Node(ST.identifier, 2, value: 'isAPlaceholder'),
        Node(ST.closeBrace, 17, value: '}'),
      ]),
    );
  });

  testWithoutContext('relaxed lexer complex', () {
    const String message =
        '{ notPlaceholder } {count,plural, =0{Hello} =1{Hello World} =2{Hello two worlds} few{Hello {count} worlds} many{Hello all {count} worlds} other{Hello other {count} worlds}}';
    final List<Node> tokens =
        Parser('string', 'app_en.arb', message, placeholders: <String>['count']).lexIntoTokens();
    expect(tokens[0].type, equals(ST.string));
  });

  testWithoutContext('parser basic', () {
    expect(
      Parser('helloWorld', 'app_en.arb', 'Hello {name}').parse(),
      equals(
        Node(
          ST.message,
          0,
          children: <Node>[
            Node(ST.string, 0, value: 'Hello '),
            Node(
              ST.placeholderExpr,
              6,
              children: <Node>[
                Node(ST.openBrace, 6, value: '{'),
                Node(ST.identifier, 7, value: 'name'),
                Node(ST.closeBrace, 11, value: '}'),
              ],
            ),
          ],
        ),
      ),
    );

    expect(
      Parser('argumentTest', 'app_en.arb', 'Today is {date, date, ::yMMd}').parse(),
      equals(
        Node(
          ST.message,
          0,
          children: <Node>[
            Node(ST.string, 0, value: 'Today is '),
            Node(
              ST.argumentExpr,
              9,
              children: <Node>[
                Node(ST.openBrace, 9, value: '{'),
                Node(ST.identifier, 10, value: 'date'),
                Node(ST.comma, 14, value: ','),
                Node(ST.argType, 16, children: <Node>[Node(ST.date, 16, value: 'date')]),
                Node(ST.comma, 20, value: ','),
                Node(ST.colon, 22, value: ':'),
                Node(ST.colon, 23, value: ':'),
                Node(ST.identifier, 24, value: 'yMMd'),
                Node(ST.closeBrace, 28, value: '}'),
              ],
            ),
          ],
        ),
      ),
    );

    expect(
      Parser(
        'plural',
        'app_en.arb',
        'There are {count} {count, plural, =1{cat} other{cats}}',
      ).parse(),
      equals(
        Node(
          ST.message,
          0,
          children: <Node>[
            Node(ST.string, 0, value: 'There are '),
            Node(
              ST.placeholderExpr,
              10,
              children: <Node>[
                Node(ST.openBrace, 10, value: '{'),
                Node(ST.identifier, 11, value: 'count'),
                Node(ST.closeBrace, 16, value: '}'),
              ],
            ),
            Node(ST.string, 17, value: ' '),
            Node(
              ST.pluralExpr,
              18,
              children: <Node>[
                Node(ST.openBrace, 18, value: '{'),
                Node(ST.identifier, 19, value: 'count'),
                Node(ST.comma, 24, value: ','),
                Node(ST.plural, 26, value: 'plural'),
                Node(ST.comma, 32, value: ','),
                Node(
                  ST.pluralParts,
                  34,
                  children: <Node>[
                    Node(
                      ST.pluralPart,
                      34,
                      children: <Node>[
                        Node(ST.equalSign, 34, value: '='),
                        Node(ST.number, 35, value: '1'),
                        Node(ST.openBrace, 36, value: '{'),
                        Node(ST.message, 37, children: <Node>[Node(ST.string, 37, value: 'cat')]),
                        Node(ST.closeBrace, 40, value: '}'),
                      ],
                    ),
                    Node(
                      ST.pluralPart,
                      42,
                      children: <Node>[
                        Node(ST.other, 42, value: 'other'),
                        Node(ST.openBrace, 47, value: '{'),
                        Node(ST.message, 48, children: <Node>[Node(ST.string, 48, value: 'cats')]),
                        Node(ST.closeBrace, 52, value: '}'),
                      ],
                    ),
                  ],
                ),
                Node(ST.closeBrace, 53, value: '}'),
              ],
            ),
          ],
        ),
      ),
    );

    expect(
      Parser('gender', 'app_en.arb', '{gender, select, male{he} female{she} other{they}}').parse(),
      equals(
        Node(
          ST.message,
          0,
          children: <Node>[
            Node(
              ST.selectExpr,
              0,
              children: <Node>[
                Node(ST.openBrace, 0, value: '{'),
                Node(ST.identifier, 1, value: 'gender'),
                Node(ST.comma, 7, value: ','),
                Node(ST.select, 9, value: 'select'),
                Node(ST.comma, 15, value: ','),
                Node(
                  ST.selectParts,
                  17,
                  children: <Node>[
                    Node(
                      ST.selectPart,
                      17,
                      children: <Node>[
                        Node(ST.identifier, 17, value: 'male'),
                        Node(ST.openBrace, 21, value: '{'),
                        Node(ST.message, 22, children: <Node>[Node(ST.string, 22, value: 'he')]),
                        Node(ST.closeBrace, 24, value: '}'),
                      ],
                    ),
                    Node(
                      ST.selectPart,
                      26,
                      children: <Node>[
                        Node(ST.identifier, 26, value: 'female'),
                        Node(ST.openBrace, 32, value: '{'),
                        Node(ST.message, 33, children: <Node>[Node(ST.string, 33, value: 'she')]),
                        Node(ST.closeBrace, 36, value: '}'),
                      ],
                    ),
                    Node(
                      ST.selectPart,
                      38,
                      children: <Node>[
                        Node(ST.other, 38, value: 'other'),
                        Node(ST.openBrace, 43, value: '{'),
                        Node(ST.message, 44, children: <Node>[Node(ST.string, 44, value: 'they')]),
                        Node(ST.closeBrace, 48, value: '}'),
                      ],
                    ),
                  ],
                ),
                Node(ST.closeBrace, 49, value: '}'),
              ],
            ),
          ],
        ),
      ),
    );
  });

  testWithoutContext('parser escaping', () {
    expect(
      Parser('escaping', 'app_en.arb', "Flutter''s amazing!", useEscaping: true).parse(),
      equals(
        Node(
          ST.message,
          0,
          children: <Node>[
            Node(ST.string, 0, value: 'Flutter'),
            Node(ST.string, 7, value: "'"),
            Node(ST.string, 9, value: 's amazing!'),
          ],
        ),
      ),
    );

    expect(
      Parser('escaping', 'app_en.arb', "'Flutter''s amazing!'", useEscaping: true).parse(),
      equals(
        Node(
          ST.message,
          0,
          children: <Node>[
            Node(ST.string, 0, value: 'Flutter'),
            Node(ST.string, 9, value: "'s amazing!"),
          ],
        ),
      ),
    );
  });

  testWithoutContext('parser recursive', () {
    expect(
      Parser(
        'pluralGender',
        'app_en.arb',
        '{count, plural, =1{{gender, select, male{he} female{she} other{they}}} other{they}}',
      ).parse(),
      equals(
        Node(
          ST.message,
          0,
          children: <Node>[
            Node(
              ST.pluralExpr,
              0,
              children: <Node>[
                Node(ST.openBrace, 0, value: '{'),
                Node(ST.identifier, 1, value: 'count'),
                Node(ST.comma, 6, value: ','),
                Node(ST.plural, 8, value: 'plural'),
                Node(ST.comma, 14, value: ','),
                Node(
                  ST.pluralParts,
                  16,
                  children: <Node>[
                    Node(
                      ST.pluralPart,
                      16,
                      children: <Node>[
                        Node(ST.equalSign, 16, value: '='),
                        Node(ST.number, 17, value: '1'),
                        Node(ST.openBrace, 18, value: '{'),
                        Node(
                          ST.message,
                          19,
                          children: <Node>[
                            Node(
                              ST.selectExpr,
                              19,
                              children: <Node>[
                                Node(ST.openBrace, 19, value: '{'),
                                Node(ST.identifier, 20, value: 'gender'),
                                Node(ST.comma, 26, value: ','),
                                Node(ST.select, 28, value: 'select'),
                                Node(ST.comma, 34, value: ','),
                                Node(
                                  ST.selectParts,
                                  36,
                                  children: <Node>[
                                    Node(
                                      ST.selectPart,
                                      36,
                                      children: <Node>[
                                        Node(ST.identifier, 36, value: 'male'),
                                        Node(ST.openBrace, 40, value: '{'),
                                        Node(
                                          ST.message,
                                          41,
                                          children: <Node>[Node(ST.string, 41, value: 'he')],
                                        ),
                                        Node(ST.closeBrace, 43, value: '}'),
                                      ],
                                    ),
                                    Node(
                                      ST.selectPart,
                                      45,
                                      children: <Node>[
                                        Node(ST.identifier, 45, value: 'female'),
                                        Node(ST.openBrace, 51, value: '{'),
                                        Node(
                                          ST.message,
                                          52,
                                          children: <Node>[Node(ST.string, 52, value: 'she')],
                                        ),
                                        Node(ST.closeBrace, 55, value: '}'),
                                      ],
                                    ),
                                    Node(
                                      ST.selectPart,
                                      57,
                                      children: <Node>[
                                        Node(ST.other, 57, value: 'other'),
                                        Node(ST.openBrace, 62, value: '{'),
                                        Node(
                                          ST.message,
                                          63,
                                          children: <Node>[Node(ST.string, 63, value: 'they')],
                                        ),
                                        Node(ST.closeBrace, 67, value: '}'),
                                      ],
                                    ),
                                  ],
                                ),
                                Node(ST.closeBrace, 68, value: '}'),
                              ],
                            ),
                          ],
                        ),
                        Node(ST.closeBrace, 69, value: '}'),
                      ],
                    ),
                    Node(
                      ST.pluralPart,
                      71,
                      children: <Node>[
                        Node(ST.other, 71, value: 'other'),
                        Node(ST.openBrace, 76, value: '{'),
                        Node(ST.message, 77, children: <Node>[Node(ST.string, 77, value: 'they')]),
                        Node(ST.closeBrace, 81, value: '}'),
                      ],
                    ),
                  ],
                ),
                Node(ST.closeBrace, 82, value: '}'),
              ],
            ),
          ],
        ),
      ),
    );
  });

  testWithoutContext('parser unexpected token', () {
    // unexpected token
    const String expectedError1 = '''
[app_en.arb:unexpectedToken] ICU Syntax Error: Expected "}" but found "=".
    { placeholder =
                  ^''';
    expect(
      () => Parser('unexpectedToken', 'app_en.arb', '{ placeholder =').parseIntoTree(),
      throwsA(
        isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains(expectedError1),
        ),
      ),
    );

    const String expectedError2 = '''
[app_en.arb:unexpectedToken] ICU Syntax Error: Expected "number" but found "}".
    { count, plural, = }
                       ^''';
    expect(
      () => Parser('unexpectedToken', 'app_en.arb', '{ count, plural, = }').parseIntoTree(),
      throwsA(
        isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains(expectedError2),
        ),
      ),
    );

    const String expectedError3 = '''
[app_en.arb:unexpectedToken] ICU Syntax Error: Expected "identifier" but found ",".
    { , plural , = }
      ^''';
    expect(
      () => Parser('unexpectedToken', 'app_en.arb', '{ , plural , = }').parseIntoTree(),
      throwsA(
        isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains(expectedError3),
        ),
      ),
    );
  });

  testWithoutContext('parser allows select cases with numbers', () {
    final Node node =
        Parser(
          'numberSelect',
          'app_en.arb',
          '{ count, select, 0{none} 100{perfect} other{required!} }',
        ).parse();
    final Node selectExpr = node.children[0];
    final Node selectParts = selectExpr.children[5];
    final Node selectPart = selectParts.children[0];
    expect(selectPart.children[0].value, equals('0'));
    expect(selectPart.children[1].value, equals('{'));
    expect(selectPart.children[2].type, equals(ST.message));
    expect(selectPart.children[3].value, equals('}'));
  });
}
