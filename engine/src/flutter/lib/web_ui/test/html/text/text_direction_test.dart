// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../paragraph/helper.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  group('$BidiFragmenter', () {
    test('empty string', () {
      expect(split(''), <_Bidi>[_Bidi('', null, ffPrevious)]);
    });

    test('basic cases', () {
      expect(split('Lorem 11 $rtlWord1  22 ipsum'), <_Bidi>[
        _Bidi('Lorem', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('11', ltr, ffPrevious),
        _Bidi(' ', null, ffSandwich),
        _Bidi(rtlWord1, rtl, ffRtl),
        _Bidi('  ', null, ffSandwich),
        _Bidi('22', ltr, ffPrevious),
        _Bidi(' ', null, ffSandwich),
        _Bidi('ipsum', ltr, ffLtr),
      ]);
    });

    test('text and digits', () {
      expect(split('Lorem11 ${rtlWord1}22 33ipsum44dolor ${rtlWord2}55$rtlWord1'), <_Bidi>[
        _Bidi('Lorem11', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi(rtlWord1, rtl, ffRtl),
        _Bidi('22', ltr, ffPrevious),
        _Bidi(' ', null, ffSandwich),
        _Bidi('33ipsum44dolor', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi(rtlWord2, rtl, ffRtl),
        _Bidi('55', ltr, ffPrevious),
        _Bidi(rtlWord1, rtl, ffRtl),
      ]);
    });

    test('Mashriqi digits', () {
      expect(split('foo ١١ ٢٢ bar'), <_Bidi>[
        _Bidi('foo', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('١١', ltr, ffRtl),
        _Bidi(' ', null, ffSandwich),
        _Bidi('٢٢', ltr, ffRtl),
        _Bidi(' ', null, ffSandwich),
        _Bidi('bar', ltr, ffLtr),
      ]);

      expect(split('$rtlWord1 ١١ ٢٢ $rtlWord2'), <_Bidi>[
        _Bidi(rtlWord1, rtl, ffRtl),
        _Bidi(' ', null, ffSandwich),
        _Bidi('١١', ltr, ffRtl),
        _Bidi(' ', null, ffSandwich),
        _Bidi('٢٢', ltr, ffRtl),
        _Bidi(' ', null, ffSandwich),
        _Bidi(rtlWord2, rtl, ffRtl),
      ]);
    });

    test('spaces', () {
      expect(split('    '), <_Bidi>[_Bidi('    ', null, ffSandwich)]);
    });

    test('symbols', () {
      expect(split('Calculate 2.2 + 4.5 and write the result'), <_Bidi>[
        _Bidi('Calculate', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('2', ltr, ffPrevious),
        _Bidi('.', null, ffSandwich),
        _Bidi('2', ltr, ffPrevious),
        _Bidi(' + ', null, ffSandwich),
        _Bidi('4', ltr, ffPrevious),
        _Bidi('.', null, ffSandwich),
        _Bidi('5', ltr, ffPrevious),
        _Bidi(' ', null, ffSandwich),
        _Bidi('and', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('write', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('the', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('result', ltr, ffLtr),
      ]);

      expect(split('Calculate $rtlWord1 2.2 + 4.5 and write the result'), <_Bidi>[
        _Bidi('Calculate', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi(rtlWord1, rtl, ffRtl),
        _Bidi(' ', null, ffSandwich),
        _Bidi('2', ltr, ffPrevious),
        _Bidi('.', null, ffSandwich),
        _Bidi('2', ltr, ffPrevious),
        _Bidi(' + ', null, ffSandwich),
        _Bidi('4', ltr, ffPrevious),
        _Bidi('.', null, ffSandwich),
        _Bidi('5', ltr, ffPrevious),
        _Bidi(' ', null, ffSandwich),
        _Bidi('and', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('write', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('the', ltr, ffLtr),
        _Bidi(' ', null, ffSandwich),
        _Bidi('result', ltr, ffLtr),
      ]);

      expect(split('12 + 24 = 36'), <_Bidi>[
        _Bidi('12', ltr, ffPrevious),
        _Bidi(' + ', null, ffSandwich),
        _Bidi('24', ltr, ffPrevious),
        _Bidi(' = ', null, ffSandwich),
        _Bidi('36', ltr, ffPrevious),
      ]);
    });

    test('handles new lines', () {
      expect(split('Lorem\n12\nipsum  \n'), <_Bidi>[
        _Bidi('Lorem', ltr, ffLtr),
        _Bidi('\n', null, ffSandwich),
        _Bidi('12', ltr, ffPrevious),
        _Bidi('\n', null, ffSandwich),
        _Bidi('ipsum', ltr, ffLtr),
        _Bidi('  \n', null, ffSandwich),
      ]);

      expect(split('$rtlWord1\n  $rtlWord2 \n'), <_Bidi>[
        _Bidi(rtlWord1, rtl, ffRtl),
        _Bidi('\n  ', null, ffSandwich),
        _Bidi(rtlWord2, rtl, ffRtl),
        _Bidi(' \n', null, ffSandwich),
      ]);
    });

    test('surrogates', () {
      expect(split('A\u{1F600}'), <_Bidi>[
        _Bidi('A', ltr, ffLtr),
        _Bidi('\u{1F600}', null, ffSandwich),
      ]);
    });
  });
}

/// Holds information about how a bidi region was split from a string.
class _Bidi {
  _Bidi(this.text, this.textDirection, this.fragmentFlow);

  factory _Bidi.fromBidiFragment(String text, BidiFragment bidiFragment) {
    return _Bidi(
      text.substring(bidiFragment.start, bidiFragment.end),
      bidiFragment.textDirection,
      bidiFragment.fragmentFlow,
    );
  }

  final String text;
  final TextDirection? textDirection;
  final FragmentFlow fragmentFlow;

  @override
  int get hashCode => Object.hash(text, textDirection);

  @override
  bool operator ==(Object other) {
    return other is _Bidi &&
        other.text == text &&
        other.textDirection == textDirection &&
        other.fragmentFlow == fragmentFlow;
  }

  @override
  String toString() {
    return '"$text" ($textDirection | $fragmentFlow)';
  }
}

List<_Bidi> split(String text) {
  return <_Bidi>[
    for (final BidiFragment bidiFragment in BidiFragmenter(text).fragment())
      _Bidi.fromBidiFragment(text, bidiFragment),
  ];
}
