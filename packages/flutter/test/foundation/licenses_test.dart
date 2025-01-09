// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('LicenseEntryWithLineBreaks - most cases', () {
    // There's some trailing spaces in this string.
    // To avoid IDEs stripping them, I've escaped them as \u0020.
    final List<LicenseParagraph> paragraphs =
        const LicenseEntryWithLineBreaks(<String>[], '''
A
A
A
  B
B
B
   C
C
C
                  D
D
D

E
E
 F
  G
 G
G

[H
 H
 H]
\u0020\u0020
I\u000cJ
\u000cK
K
\u000c
L
L L
L  L
L   L
L    L
L     L

   M
M\u0020\u0020\u0020
M\u0020\u0020\u0020\u0020

N

O
O


P



QQQ

RR RRR RRRR RRRRR
R

S

   T

      U
         V

        W

       X
\u0020\u0020\u0020\u0020\u0020\u0020
      Y''').paragraphs.toList();

    int index = 0;
    expect(paragraphs[index].text, 'A A A');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'B B B');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'C C C');
    expect(paragraphs[index].indent, 1);
    index += 1;
    expect(paragraphs[index].text, 'D D D');
    expect(paragraphs[index].indent, LicenseParagraph.centeredIndent);
    index += 1;
    expect(paragraphs[index].text, 'E E');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'F');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'G G G');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, '[H H H]');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'I');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'J');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'K K');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'L L L L  L L   L L    L L     L');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'M M    M    ');
    expect(paragraphs[index].indent, 1);
    index += 1;
    expect(paragraphs[index].text, 'N');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'O O');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'P');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'QQQ');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'RR RRR RRRR RRRRR R');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'S');
    expect(paragraphs[index].indent, 0);
    index += 1;
    expect(paragraphs[index].text, 'T');
    expect(paragraphs[index].indent, 1);
    index += 1;
    expect(paragraphs[index].text, 'U');
    expect(paragraphs[index].indent, 2);
    index += 1;
    expect(paragraphs[index].text, 'V');
    expect(paragraphs[index].indent, 3);
    index += 1;
    expect(paragraphs[index].text, 'W');
    expect(paragraphs[index].indent, 2);
    index += 1;
    expect(paragraphs[index].text, 'X');
    expect(paragraphs[index].indent, 2);
    index += 1;
    expect(paragraphs[index].text, 'Y');
    expect(paragraphs[index].indent, 2);
    index += 1;
    expect(paragraphs, hasLength(index));
  });

  test('LicenseEntryWithLineBreaks - leading and trailing whitespace', () {
    expect(
      const LicenseEntryWithLineBreaks(<String>[], '    \n\n    ').paragraphs.toList(),
      isEmpty,
    );
    expect(
      const LicenseEntryWithLineBreaks(<String>[], '    \r\n\r\n    ').paragraphs.toList(),
      isEmpty,
    );

    List<LicenseParagraph> paragraphs;

    paragraphs = const LicenseEntryWithLineBreaks(<String>[], '    \nA\n    ').paragraphs.toList();
    expect(paragraphs[0].text, 'A');
    expect(paragraphs[0].indent, 0);
    expect(paragraphs, hasLength(1));

    paragraphs = const LicenseEntryWithLineBreaks(<String>[], '\n\n\nA\n\n\n').paragraphs.toList();
    expect(paragraphs[0].text, 'A');
    expect(paragraphs[0].indent, 0);
    expect(paragraphs, hasLength(1));
  });

  test('LicenseRegistry', () async {
    expect(await LicenseRegistry.licenses.toList(), isEmpty);
    LicenseRegistry.addLicense(() async* {
      yield const LicenseEntryWithLineBreaks(<String>[], 'A');
      yield const LicenseEntryWithLineBreaks(<String>[], 'B');
    });
    LicenseRegistry.addLicense(() async* {
      yield const LicenseEntryWithLineBreaks(<String>[], 'C');
      yield const LicenseEntryWithLineBreaks(<String>[], 'D');
    });
    expect(await LicenseRegistry.licenses.toList(), hasLength(4));
    final List<LicenseEntry> licenses = await LicenseRegistry.licenses.toList();
    expect(licenses, hasLength(4));
    expect(licenses[0].paragraphs.single.text, 'A');
    expect(licenses[1].paragraphs.single.text, 'B');
    expect(licenses[2].paragraphs.single.text, 'C');
    expect(licenses[3].paragraphs.single.text, 'D');
  });
}
