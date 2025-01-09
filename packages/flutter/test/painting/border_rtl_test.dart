// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart' show DiagnosticLevel, FlutterError;
import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

class SillyBorder extends BoxBorder {
  const SillyBorder();

  @override
  dynamic noSuchMethod(Invocation invocation) => null;
}

bool _sideMatches(BorderSide x, BorderSide y) {
  const double limit = 1 / 255;
  return (x.color.a - y.color.a).abs() < limit &&
      (x.color.r - y.color.r).abs() < limit &&
      (x.color.g - y.color.g).abs() < limit &&
      (x.color.b - y.color.b).abs() < limit &&
      x.width == y.width &&
      x.style == y.style &&
      x.strokeAlign == y.strokeAlign;
}

class _BorderMatches extends Matcher {
  _BorderMatches(this._target);

  final dynamic _target;

  @override
  Description describe(Description description) {
    description.add('expected $_target');
    return description;
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    return item is Border &&
        _target is Border &&
        _sideMatches(item.top, _target.top) &&
        _sideMatches(item.right, _target.right) &&
        _sideMatches(item.bottom, _target.bottom) &&
        _sideMatches(item.left, _target.left);
  }
}

Matcher _matchesBorder(dynamic border) => _BorderMatches(border);

class _BorderDirectionalMatches extends Matcher {
  _BorderDirectionalMatches(this._target);

  final dynamic _target;

  @override
  Description describe(Description description) {
    description.add('expected $_target');
    return description;
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    return item is BorderDirectional &&
        _target is BorderDirectional &&
        _sideMatches(item.top, _target.top) &&
        _sideMatches(item.start, _target.start) &&
        _sideMatches(item.bottom, _target.bottom) &&
        _sideMatches(item.end, _target.end);
  }
}

Matcher _matchesBorderDirectional(dynamic border) => _BorderDirectionalMatches(border);

void main() {
  test('BoxBorder.lerp', () {
    // names of the form fooAtX are foo, lerped X% of the way to null
    const BoxBorder directionalWithMagentaTop5 = BorderDirectional(
      top: BorderSide(color: Color(0xFFFF00FF), width: 5.0),
    );
    const BoxBorder directionalWithMagentaTop5At25 = BorderDirectional(
      top: BorderSide(color: Color(0x3F3F003F), width: 1.25),
    );
    const BoxBorder directionalWithMagentaTop5At75 = BorderDirectional(
      top: BorderSide(color: Color(0xBFBF00BF), width: 3.75),
    );
    const BoxBorder directionalWithSides10 = BorderDirectional(
      start: BorderSide(width: 10.0),
      end: BorderSide(width: 20.0),
    );
    const BoxBorder directionalWithSides10At25 = BorderDirectional(
      start: BorderSide(width: 2.5, color: Color(0x3F000000)),
      end: BorderSide(width: 5.0, color: Color(0x3F000000)),
    );
    const BoxBorder directionalWithSides10At50 = BorderDirectional(
      start: BorderSide(width: 5.0, color: Color(0x7F000000)),
      end: BorderSide(width: 10.0, color: Color(0x7F000000)),
    );
    const BoxBorder directionalWithSides10At75 = BorderDirectional(
      start: BorderSide(width: 7.5, color: Color(0xBF000000)),
      end: BorderSide(width: 15.0, color: Color(0xBF000000)),
    );
    const BoxBorder directionalWithSides20 = BorderDirectional(
      start: BorderSide(width: 20.0),
      end: BorderSide(width: 40.0),
    );
    const BoxBorder directionalWithSides30 = BorderDirectional(
      start: BorderSide(width: 30.0),
      end: BorderSide(width: 60.0),
    );
    const BoxBorder directionalWithTop10 = BorderDirectional(top: BorderSide(width: 10.0));
    const BoxBorder directionalWithYellowTop10 = BorderDirectional(
      top: BorderSide(color: Color(0xFFFFFF00), width: 10.0),
    );
    const BoxBorder directionalWithYellowTop5 = BorderDirectional(
      top: BorderSide(color: Color(0xFFFFFF00), width: 5.0),
    );
    const BoxBorder visualWithMagentaTop10 = Border(
      top: BorderSide(color: Color(0xFFFF00FF), width: 10.0),
    );
    const BoxBorder visualWithMagentaTop5 = Border(
      top: BorderSide(color: Color(0xFFFF00FF), width: 5.0),
    );
    const BoxBorder visualWithSides10 = Border(
      left: BorderSide(width: 10.0),
      right: BorderSide(width: 20.0),
    );
    const BoxBorder visualWithSides10At25 = Border(
      left: BorderSide(width: 2.5, color: Color(0x3F000000)),
      right: BorderSide(width: 5.0, color: Color(0x3F000000)),
    );
    const BoxBorder visualWithSides10At50 = Border(
      left: BorderSide(width: 5.0, color: Color(0x7F000000)),
      right: BorderSide(width: 10.0, color: Color(0x7F000000)),
    );
    const BoxBorder visualWithSides10At75 = Border(
      left: BorderSide(width: 7.5, color: Color(0xBF000000)),
      right: BorderSide(width: 15.0, color: Color(0xBF000000)),
    );
    const BoxBorder visualWithSides20 = Border(
      left: BorderSide(width: 20.0),
      right: BorderSide(width: 40.0),
    );
    const BoxBorder visualWithSides30 = Border(
      left: BorderSide(width: 30.0),
      right: BorderSide(width: 60.0),
    );
    const BoxBorder visualWithTop10 = Border(top: BorderSide(width: 10.0));
    const BoxBorder visualWithTop100 = Border(top: BorderSide(width: 100.0));
    const BoxBorder visualWithTop190 = Border(top: BorderSide(width: 190.0));
    const BoxBorder visualWithYellowTop5 = Border(
      top: BorderSide(color: Color(0xFFFFFF00), width: 5.0),
    );
    const BoxBorder visualWithYellowTop5At25 = Border(
      top: BorderSide(color: Color(0x3F3F3F00), width: 1.25),
    );
    const BoxBorder visualWithYellowTop5At75 = Border(
      top: BorderSide(color: Color(0xBFBFBF00), width: 3.75),
    );

    expect(BoxBorder.lerp(null, null, -1.0), null);
    expect(BoxBorder.lerp(Border.all(width: 10.0), null, -1.0), Border.all(width: 20.0));
    expect(
      BoxBorder.lerp(null, Border.all(width: 10.0), -1.0),
      Border.all(width: 0.0, style: BorderStyle.none),
    );
    expect(
      BoxBorder.lerp(directionalWithTop10, null, -1.0),
      const BorderDirectional(top: BorderSide(width: 20.0)),
    );
    expect(BoxBorder.lerp(null, directionalWithTop10, -1.0), const BorderDirectional());
    expect(BoxBorder.lerp(directionalWithTop10, visualWithTop100, -1.0), const Border());
    expect(BoxBorder.lerp(visualWithSides10, directionalWithMagentaTop5, -1.0), visualWithSides20);
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithMagentaTop5, -1.0),
      const Border(top: BorderSide(color: Color(0xFFFFFF00), width: 5.0)),
    );
    expect(BoxBorder.lerp(visualWithSides10, directionalWithSides10, -1.0), visualWithSides30);
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithSides10, -1.0),
      directionalWithYellowTop10,
    );
    expect(() => BoxBorder.lerp(const SillyBorder(), const Border(), -1.0), throwsFlutterError);

    expect(BoxBorder.lerp(null, null, 0.0), null);
    expect(BoxBorder.lerp(Border.all(width: 10.0), null, 0.0), Border.all(width: 10.0));
    expect(BoxBorder.lerp(null, Border.all(width: 10.0), 0.0), const Border());
    expect(
      BoxBorder.lerp(directionalWithTop10, null, 0.0),
      const BorderDirectional(top: BorderSide(width: 10.0)),
    );
    expect(BoxBorder.lerp(null, directionalWithTop10, 0.0), const BorderDirectional());
    expect(BoxBorder.lerp(directionalWithTop10, visualWithTop100, 0.0), visualWithTop10);
    expect(BoxBorder.lerp(visualWithSides10, directionalWithMagentaTop5, 0.0), visualWithSides10);
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithMagentaTop5, 0.0),
      const Border(top: BorderSide(color: Color(0xFFFFFF00), width: 5.0)),
    );
    expect(BoxBorder.lerp(visualWithSides10, directionalWithSides10, 0.0), visualWithSides10);
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithSides10, 0.0),
      directionalWithYellowTop5,
    );
    expect(() => BoxBorder.lerp(const SillyBorder(), const Border(), 0.0), throwsFlutterError);

    expect(BoxBorder.lerp(null, null, 0.25), null);
    expect(BoxBorder.lerp(Border.all(width: 10.0), null, 0.25), Border.all(width: 7.5));
    expect(BoxBorder.lerp(null, Border.all(width: 10.0), 0.25), Border.all(width: 2.5));
    expect(
      BoxBorder.lerp(directionalWithTop10, null, 0.25),
      const BorderDirectional(top: BorderSide(width: 7.5)),
    );
    expect(
      BoxBorder.lerp(null, directionalWithTop10, 0.25),
      const BorderDirectional(top: BorderSide(width: 2.5)),
    );
    expect(
      BoxBorder.lerp(directionalWithTop10, visualWithTop100, 0.25),
      const Border(top: BorderSide(width: 32.5)),
    );
    expect(
      BoxBorder.lerp(visualWithSides10, directionalWithMagentaTop5, 0.25),
      _matchesBorder(visualWithSides10At75 + directionalWithMagentaTop5At25),
    );
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithMagentaTop5, 0.25),
      Border(
        top: BorderSide(
          width: 5.0,
          color: Color.lerp(const Color(0xFFFFFF00), const Color(0xFFFF00FF), 0.25)!,
        ),
      ),
    );
    expect(
      BoxBorder.lerp(visualWithSides10, directionalWithSides10, 0.25),
      _matchesBorder(visualWithSides10At50),
    );
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithSides10, 0.25),
      _matchesBorderDirectional(visualWithYellowTop5At75 + directionalWithSides10At25),
    );
    expect(() => BoxBorder.lerp(const SillyBorder(), const Border(), 0.25), throwsFlutterError);

    expect(BoxBorder.lerp(null, null, 0.75), null);
    expect(BoxBorder.lerp(Border.all(width: 10.0), null, 0.75), Border.all(width: 2.5));
    expect(BoxBorder.lerp(null, Border.all(width: 10.0), 0.75), Border.all(width: 7.5));
    expect(
      BoxBorder.lerp(directionalWithTop10, null, 0.75),
      const BorderDirectional(top: BorderSide(width: 2.5)),
    );
    expect(
      BoxBorder.lerp(null, directionalWithTop10, 0.75),
      const BorderDirectional(top: BorderSide(width: 7.5)),
    );
    expect(
      BoxBorder.lerp(directionalWithTop10, visualWithTop100, 0.75),
      const Border(top: BorderSide(width: 77.5)),
    );
    expect(
      BoxBorder.lerp(visualWithSides10, directionalWithMagentaTop5, 0.75),
      _matchesBorder(visualWithSides10At25 + directionalWithMagentaTop5At75),
    );
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithMagentaTop5, 0.75),
      Border(
        top: BorderSide(
          width: 5.0,
          color: Color.lerp(const Color(0xFFFFFF00), const Color(0xFFFF00FF), 0.75)!,
        ),
      ),
    );
    expect(
      BoxBorder.lerp(visualWithSides10, directionalWithSides10, 0.75),
      _matchesBorderDirectional(directionalWithSides10At50),
    );
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithSides10, 0.75),
      _matchesBorderDirectional(visualWithYellowTop5At25 + directionalWithSides10At75),
    );
    expect(() => BoxBorder.lerp(const SillyBorder(), const Border(), 0.75), throwsFlutterError);

    expect(BoxBorder.lerp(null, null, 1.0), null);
    expect(
      BoxBorder.lerp(Border.all(width: 10.0), null, 1.0),
      Border.all(width: 0.0, style: BorderStyle.none),
    );
    expect(BoxBorder.lerp(null, Border.all(width: 10.0), 1.0), Border.all(width: 10.0));
    expect(BoxBorder.lerp(directionalWithTop10, null, 1.0), const BorderDirectional());
    expect(
      BoxBorder.lerp(null, directionalWithTop10, 1.0),
      const BorderDirectional(top: BorderSide(width: 10.0)),
    );
    expect(BoxBorder.lerp(directionalWithTop10, visualWithTop100, 1.0), visualWithTop100);
    expect(
      BoxBorder.lerp(visualWithSides10, directionalWithMagentaTop5, 1.0),
      visualWithMagentaTop5,
    );
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithMagentaTop5, 1.0),
      visualWithMagentaTop5,
    );
    expect(BoxBorder.lerp(visualWithSides10, directionalWithSides10, 1.0), directionalWithSides10);
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithSides10, 1.0),
      directionalWithSides10,
    );
    expect(() => BoxBorder.lerp(const SillyBorder(), const Border(), 1.0), throwsFlutterError);

    expect(BoxBorder.lerp(null, null, 2.0), null);
    expect(
      BoxBorder.lerp(Border.all(width: 10.0), null, 2.0),
      Border.all(width: 0.0, style: BorderStyle.none),
    );
    expect(BoxBorder.lerp(null, Border.all(width: 10.0), 2.0), Border.all(width: 20.0));
    expect(BoxBorder.lerp(directionalWithTop10, null, 2.0), const BorderDirectional());
    expect(
      BoxBorder.lerp(null, directionalWithTop10, 2.0),
      const BorderDirectional(top: BorderSide(width: 20.0)),
    );
    expect(BoxBorder.lerp(directionalWithTop10, visualWithTop100, 2.0), visualWithTop190);
    expect(
      BoxBorder.lerp(visualWithSides10, directionalWithMagentaTop5, 2.0),
      visualWithMagentaTop10,
    );
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithMagentaTop5, 2.0),
      visualWithMagentaTop5,
    );
    expect(BoxBorder.lerp(visualWithSides10, directionalWithSides10, 2.0), directionalWithSides30);
    expect(
      BoxBorder.lerp(visualWithYellowTop5, directionalWithSides10, 2.0),
      directionalWithSides20,
    );
    expect(() => BoxBorder.lerp(const SillyBorder(), const Border(), 2.0), throwsFlutterError);
  });

  test('BoxBorder.lerp throws correct FlutterError message', () {
    late FlutterError error;
    try {
      BoxBorder.lerp(const SillyBorder(), const Border(), 2.0);
    } on FlutterError catch (e) {
      error = e;
    }
    expect(error, isNotNull);
    expect(error.diagnostics.length, 3);
    expect(error.diagnostics[2].level, DiagnosticLevel.hint);
    expect(
      error.diagnostics[2].toStringDeep(),
      equalsIgnoringHashCodes(
        'For a more general interpolation method, consider using\n'
        'ShapeBorder.lerp instead.\n',
      ),
    );
    expect(
      error.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   BoxBorder.lerp can only interpolate Border and BorderDirectional\n'
        '   classes.\n'
        '   BoxBorder.lerp() was called with two objects of type SillyBorder\n'
        '   and Border:\n'
        '     SillyBorder()\n'
        '     Border.all(BorderSide(width: 0.0, style: none))\n'
        '   However, only Border and BorderDirectional classes are supported\n'
        '   by this method.\n'
        '   For a more general interpolation method, consider using\n'
        '   ShapeBorder.lerp instead.\n',
      ),
    );
  });

  test('BoxBorder.getInnerPath / BoxBorder.getOuterPath', () {
    // for Border, BorderDirectional
    const Border border = Border(top: BorderSide(width: 10.0), right: BorderSide(width: 20.0));
    const BorderDirectional borderDirectional = BorderDirectional(
      top: BorderSide(width: 10.0),
      end: BorderSide(width: 20.0),
    );
    expect(
      border.getOuterPath(
        const Rect.fromLTRB(50.0, 60.0, 110.0, 190.0),
        textDirection: TextDirection.rtl,
      ),
      isPathThat(
        includes: <Offset>[
          const Offset(50.0, 60.0),
          const Offset(60.0, 60.0),
          const Offset(60.0, 70.0),
          const Offset(80.0, 190.0),
          const Offset(109.0, 189.0),
          const Offset(110.0, 80.0),
          const Offset(110.0, 190.0),
        ],
        excludes: <Offset>[
          const Offset(40.0, 60.0),
          const Offset(50.0, 50.0),
          const Offset(111.0, 190.0),
          const Offset(110.0, 191.0),
          const Offset(111.0, 191.0),
          Offset.zero,
          const Offset(-10.0, -10.0),
          const Offset(0.0, -10.0),
          const Offset(-10.0, 0.0),
          const Offset(1000.0, 1000.0),
        ],
      ),
    );
    expect(
      border.getInnerPath(
        const Rect.fromLTRB(50.0, 60.0, 110.0, 190.0),
        textDirection: TextDirection.rtl,
      ),
      // inner path is a rect from 50.0,70.0 to 90.0,190.0
      isPathThat(
        includes: <Offset>[
          const Offset(50.0, 70.0),
          const Offset(55.0, 70.0),
          const Offset(50.0, 75.0),
          const Offset(70.0, 70.0),
          const Offset(70.0, 71.0),
          const Offset(71.0, 70.0),
          const Offset(71.0, 71.0),
          const Offset(80.0, 180.0),
          const Offset(80.0, 190.0),
          const Offset(89.0, 189.0),
          const Offset(90.0, 190.0),
        ],
        excludes: <Offset>[
          const Offset(40.0, 60.0),
          const Offset(50.0, 50.0),
          const Offset(50.0, 60.0),
          const Offset(60.0, 60.0),
          Offset.zero,
          const Offset(-10.0, -10.0),
          const Offset(0.0, -10.0),
          const Offset(-10.0, 0.0),
          const Offset(110.0, 80.0),
          const Offset(89.0, 191.0),
          const Offset(90.0, 191.0),
          const Offset(91.0, 189.0),
          const Offset(91.0, 190.0),
          const Offset(91.0, 191.0),
          const Offset(109.0, 189.0),
          const Offset(110.0, 190.0),
          const Offset(1000.0, 1000.0),
        ],
      ),
    );
    expect(
      borderDirectional.getOuterPath(
        const Rect.fromLTRB(50.0, 60.0, 110.0, 190.0),
        textDirection: TextDirection.rtl,
      ),
      isPathThat(
        includes: <Offset>[
          const Offset(50.0, 60.0),
          const Offset(60.0, 60.0),
          const Offset(60.0, 70.0),
          const Offset(80.0, 190.0),
          const Offset(109.0, 189.0),
          const Offset(110.0, 80.0),
          const Offset(110.0, 190.0),
        ],
        excludes: <Offset>[
          const Offset(40.0, 60.0),
          const Offset(50.0, 50.0),
          const Offset(111.0, 190.0),
          const Offset(110.0, 191.0),
          const Offset(111.0, 191.0),
          Offset.zero,
          const Offset(-10.0, -10.0),
          const Offset(0.0, -10.0),
          const Offset(-10.0, 0.0),
          const Offset(1000.0, 1000.0),
        ],
      ),
    );
    expect(
      borderDirectional.getInnerPath(
        const Rect.fromLTRB(50.0, 60.0, 110.0, 190.0),
        textDirection: TextDirection.rtl,
      ),
      // inner path is a rect from 70.0,70.0 to 110.0,190.0
      isPathThat(
        includes: <Offset>[
          const Offset(70.0, 70.0),
          const Offset(70.0, 71.0),
          const Offset(71.0, 70.0),
          const Offset(71.0, 71.0),
          const Offset(80.0, 180.0),
          const Offset(80.0, 190.0),
          const Offset(89.0, 189.0),
          const Offset(90.0, 190.0),
          const Offset(91.0, 189.0),
          const Offset(91.0, 190.0),
          const Offset(109.0, 189.0),
          const Offset(110.0, 80.0),
          const Offset(110.0, 190.0),
        ],
        excludes: <Offset>[
          const Offset(40.0, 60.0),
          const Offset(50.0, 50.0),
          const Offset(50.0, 60.0),
          const Offset(50.0, 70.0),
          const Offset(50.0, 75.0),
          const Offset(55.0, 70.0),
          const Offset(60.0, 60.0),
          Offset.zero,
          const Offset(-10.0, -10.0),
          const Offset(0.0, -10.0),
          const Offset(-10.0, 0.0),
          const Offset(89.0, 191.0),
          const Offset(90.0, 191.0),
          const Offset(91.0, 191.0),
          const Offset(110.0, 191.0),
          const Offset(111.0, 190.0),
          const Offset(111.0, 191.0),
          const Offset(1000.0, 1000.0),
        ],
      ),
    );
    expect(
      borderDirectional.getOuterPath(
        const Rect.fromLTRB(50.0, 60.0, 110.0, 190.0),
        textDirection: TextDirection.ltr,
      ),
      isPathThat(
        includes: <Offset>[
          const Offset(50.0, 60.0),
          const Offset(60.0, 60.0),
          const Offset(60.0, 70.0),
          const Offset(80.0, 190.0),
          const Offset(109.0, 189.0),
          const Offset(110.0, 80.0),
          const Offset(110.0, 190.0),
        ],
        excludes: <Offset>[
          const Offset(40.0, 60.0),
          const Offset(50.0, 50.0),
          const Offset(111.0, 190.0),
          const Offset(110.0, 191.0),
          const Offset(111.0, 191.0),
          Offset.zero,
          const Offset(-10.0, -10.0),
          const Offset(0.0, -10.0),
          const Offset(-10.0, 0.0),
          const Offset(1000.0, 1000.0),
        ],
      ),
    );
    expect(
      borderDirectional.getInnerPath(
        const Rect.fromLTRB(50.0, 60.0, 110.0, 190.0),
        textDirection: TextDirection.ltr,
      ),
      // inner path is a rect from 50.0,70.0 to 90.0,190.0
      isPathThat(
        includes: <Offset>[
          const Offset(50.0, 70.0),
          const Offset(50.0, 75.0),
          const Offset(55.0, 70.0),
          const Offset(70.0, 70.0),
          const Offset(70.0, 71.0),
          const Offset(71.0, 70.0),
          const Offset(71.0, 71.0),
          const Offset(80.0, 180.0),
          const Offset(80.0, 190.0),
          const Offset(89.0, 189.0),
          const Offset(90.0, 190.0),
        ],
        excludes: <Offset>[
          const Offset(50.0, 50.0),
          const Offset(40.0, 60.0),
          const Offset(50.0, 60.0),
          const Offset(60.0, 60.0),
          Offset.zero,
          const Offset(-10.0, -10.0),
          const Offset(0.0, -10.0),
          const Offset(-10.0, 0.0),
          const Offset(110.0, 80.0),
          const Offset(89.0, 191.0),
          const Offset(90.0, 191.0),
          const Offset(91.0, 189.0),
          const Offset(91.0, 190.0),
          const Offset(91.0, 191.0),
          const Offset(109.0, 189.0),
          const Offset(110.0, 190.0),
          const Offset(1000.0, 1000.0),
        ],
      ),
    );
  });

  test('BorderDirectional.merge', () {
    const BorderSide magenta3 = BorderSide(color: Color(0xFFFF00FF), width: 3.0);
    const BorderSide magenta6 = BorderSide(color: Color(0xFFFF00FF), width: 6.0);
    const BorderSide yellow2 = BorderSide(color: Color(0xFFFFFF00), width: 2.0);
    const BorderSide yellowNone0 = BorderSide(
      color: Color(0xFFFFFF00),
      width: 0.0,
      style: BorderStyle.none,
    );
    expect(
      BorderDirectional.merge(
        const BorderDirectional(top: yellow2),
        const BorderDirectional(end: magenta3),
      ),
      const BorderDirectional(top: yellow2, end: magenta3),
    );
    expect(
      BorderDirectional.merge(
        const BorderDirectional(bottom: magenta3),
        const BorderDirectional(bottom: magenta3),
      ),
      const BorderDirectional(bottom: magenta6),
    );
    expect(
      BorderDirectional.merge(
        const BorderDirectional(start: magenta3, end: yellowNone0),
        const BorderDirectional(end: yellow2),
      ),
      const BorderDirectional(start: magenta3, end: yellow2),
    );
    expect(
      BorderDirectional.merge(const BorderDirectional(), const BorderDirectional()),
      const BorderDirectional(),
    );
    expect(
      () => BorderDirectional.merge(
        const BorderDirectional(start: magenta3),
        const BorderDirectional(start: yellow2),
      ),
      throwsAssertionError,
    );
  });

  test('BorderDirectional.dimensions', () {
    expect(
      const BorderDirectional(
        top: BorderSide(width: 3.0),
        start: BorderSide(width: 2.0),
        end: BorderSide(width: 7.0),
        bottom: BorderSide(width: 5.0),
      ).dimensions,
      const EdgeInsetsDirectional.fromSTEB(2.0, 3.0, 7.0, 5.0),
    );
  });

  test('BorderDirectional.isUniform', () {
    expect(
      const BorderDirectional(
        top: BorderSide(width: 3.0),
        start: BorderSide(width: 3.0),
        end: BorderSide(width: 3.0),
        bottom: BorderSide(width: 3.1),
      ).isUniform,
      false,
    );
    expect(
      const BorderDirectional(
        top: BorderSide(width: 3.0),
        start: BorderSide(width: 3.0),
        end: BorderSide(width: 3.0),
        bottom: BorderSide(width: 3.0),
      ).isUniform,
      true,
    );
    expect(
      const BorderDirectional(
        top: BorderSide(color: Color(0xFFFFFFFF)),
        start: BorderSide(color: Color(0xFFFFFFFE)),
        end: BorderSide(color: Color(0xFFFFFFFF)),
        bottom: BorderSide(color: Color(0xFFFFFFFF)),
      ).isUniform,
      false,
    );
    expect(
      const BorderDirectional(
        top: BorderSide(color: Color(0xFFFFFFFF)),
        start: BorderSide(color: Color(0xFFFFFFFF)),
        end: BorderSide(color: Color(0xFFFFFFFF)),
        bottom: BorderSide(color: Color(0xFFFFFFFF)),
      ).isUniform,
      true,
    );
    expect(
      const BorderDirectional(
        top: BorderSide(style: BorderStyle.none),
        start: BorderSide(style: BorderStyle.none),
        end: BorderSide(style: BorderStyle.none),
        bottom: BorderSide(width: 0.0),
      ).isUniform,
      false,
    );
    expect(
      const BorderDirectional(
        top: BorderSide(style: BorderStyle.none),
        start: BorderSide(style: BorderStyle.none),
        end: BorderSide(style: BorderStyle.none),
      ).isUniform,
      false,
    );
    expect(const BorderDirectional().isUniform, true);
    expect(const BorderDirectional().isUniform, true);
  });

  test('BorderDirectional.add - all directional', () {
    const BorderSide magenta3 = BorderSide(color: Color(0xFFFF00FF), width: 3.0);
    const BorderSide magenta6 = BorderSide(color: Color(0xFFFF00FF), width: 6.0);
    const BorderSide yellow2 = BorderSide(color: Color(0xFFFFFF00), width: 2.0);
    const BorderSide yellowNone0 = BorderSide(
      color: Color(0xFFFFFF00),
      width: 0.0,
      style: BorderStyle.none,
    );
    expect(
      const BorderDirectional(top: yellow2) + const BorderDirectional(end: magenta3),
      const BorderDirectional(top: yellow2, end: magenta3),
    );
    expect(
      const BorderDirectional(bottom: magenta3) + const BorderDirectional(bottom: magenta3),
      const BorderDirectional(bottom: magenta6),
    );
    expect(
      const BorderDirectional(start: magenta3, end: yellowNone0) +
          const BorderDirectional(end: yellow2),
      const BorderDirectional(start: magenta3, end: yellow2),
    );
    expect(const BorderDirectional() + const BorderDirectional(), const BorderDirectional());
    expect(
      const BorderDirectional(start: magenta3) + const BorderDirectional(start: yellow2),
      isNot(isA<BorderDirectional>()), // see shape_border_test.dart for better tests of this case
    );
    const BorderDirectional b3 = BorderDirectional(top: magenta3);
    const BorderDirectional b6 = BorderDirectional(top: magenta6);
    expect(b3 + b3, b6);
    const BorderDirectional b0 = BorderDirectional(top: yellowNone0);
    const BorderDirectional bZ = BorderDirectional();
    expect(b0 + b0, bZ);
    expect(bZ + bZ, bZ);
    expect(b0 + bZ, bZ);
    expect(bZ + b0, bZ);
  });

  test('BorderDirectional.add', () {
    const BorderSide side1 = BorderSide(color: Color(0x11111111));
    const BorderSide doubleSide1 = BorderSide(color: Color(0x11111111), width: 2.0);
    const BorderSide side2 = BorderSide(color: Color(0x22222222));
    const BorderSide doubleSide2 = BorderSide(color: Color(0x22222222), width: 2.0);

    // adding tops and sides
    expect(
      const Border(left: side1) + const BorderDirectional(top: side2),
      const Border(left: side1, top: side2),
    );
    expect(
      const BorderDirectional(start: side1) + const Border(top: side2),
      const BorderDirectional(start: side1, top: side2),
    );
    expect(
      const Border(top: side2) + const BorderDirectional(start: side1),
      const BorderDirectional(start: side1, top: side2),
    );
    expect(
      const BorderDirectional(top: side2) + const Border(left: side1),
      const Border(left: side1, top: side2),
    );

    // adding incompatible tops and bottoms
    expect(
      (const Border(top: side1) + const BorderDirectional(top: side2)).toString(),
      contains(' + '),
    );
    expect(
      (const BorderDirectional(top: side2) + const Border(top: side1)).toString(),
      contains(' + '),
    );
    expect(
      (const Border(bottom: side1) + const BorderDirectional(bottom: side2)).toString(),
      contains(' + '),
    );
    expect(
      (const BorderDirectional(bottom: side2) + const Border(bottom: side1)).toString(),
      contains(' + '),
    );

    // adding compatible tops and bottoms
    expect(
      const BorderDirectional(top: side1) + const Border(top: side1),
      const Border(top: doubleSide1),
    );
    expect(
      const Border(top: side1) + const BorderDirectional(top: side1),
      const Border(top: doubleSide1),
    );
    expect(
      const BorderDirectional(bottom: side1) + const Border(bottom: side1),
      const Border(bottom: doubleSide1),
    );
    expect(
      const Border(bottom: side1) + const BorderDirectional(bottom: side1),
      const Border(bottom: doubleSide1),
    );

    const Border borderWithLeft = Border(left: side1, top: side2, bottom: side2);
    const Border borderWithRight = Border(right: side1, top: side2, bottom: side2);
    const Border borderWithoutSides = Border(top: side2, bottom: side2);
    const BorderDirectional borderDirectionalWithStart = BorderDirectional(
      start: side1,
      top: side2,
      bottom: side2,
    );
    const BorderDirectional borderDirectionalWithEnd = BorderDirectional(
      end: side1,
      top: side2,
      bottom: side2,
    );
    const BorderDirectional borderDirectionalWithoutSides = BorderDirectional(
      top: side2,
      bottom: side2,
    );

    expect(
      (borderWithLeft + borderDirectionalWithStart).toString(),
      '$borderWithLeft + $borderDirectionalWithStart',
    );
    expect(
      (borderWithLeft + borderDirectionalWithEnd).toString(),
      '$borderWithLeft + $borderDirectionalWithEnd',
    );
    expect(
      (borderWithLeft + borderDirectionalWithoutSides).toString(),
      '${const Border(left: side1, top: doubleSide2, bottom: doubleSide2)}',
    );
    expect(
      (borderWithRight + borderDirectionalWithStart).toString(),
      '$borderWithRight + $borderDirectionalWithStart',
    );
    expect(
      (borderWithRight + borderDirectionalWithEnd).toString(),
      '$borderWithRight + $borderDirectionalWithEnd',
    );
    expect(
      (borderWithRight + borderDirectionalWithoutSides).toString(),
      '${const Border(right: side1, top: doubleSide2, bottom: doubleSide2)}',
    );
    expect(
      (borderWithoutSides + borderDirectionalWithStart).toString(),
      '${const BorderDirectional(start: side1, top: doubleSide2, bottom: doubleSide2)}',
    );
    expect(
      (borderWithoutSides + borderDirectionalWithEnd).toString(),
      '${const BorderDirectional(end: side1, top: doubleSide2, bottom: doubleSide2)}',
    );
    expect(
      (borderWithoutSides + borderDirectionalWithoutSides).toString(),
      '${const Border(top: doubleSide2, bottom: doubleSide2)}',
    );

    expect(
      (borderDirectionalWithStart + borderWithLeft).toString(),
      '$borderDirectionalWithStart + $borderWithLeft',
    );
    expect(
      (borderDirectionalWithEnd + borderWithLeft).toString(),
      '$borderDirectionalWithEnd + $borderWithLeft',
    );
    expect(
      (borderDirectionalWithoutSides + borderWithLeft).toString(),
      '${const Border(left: side1, top: doubleSide2, bottom: doubleSide2)}',
    );
    expect(
      (borderDirectionalWithStart + borderWithRight).toString(),
      '$borderDirectionalWithStart + $borderWithRight',
    );
    expect(
      (borderDirectionalWithEnd + borderWithRight).toString(),
      '$borderDirectionalWithEnd + $borderWithRight',
    );
    expect(
      (borderDirectionalWithoutSides + borderWithRight).toString(),
      '${const Border(right: side1, top: doubleSide2, bottom: doubleSide2)}',
    );
    expect(
      (borderDirectionalWithStart + borderWithoutSides).toString(),
      '${const BorderDirectional(start: side1, top: doubleSide2, bottom: doubleSide2)}',
    );
    expect(
      (borderDirectionalWithEnd + borderWithoutSides).toString(),
      '${const BorderDirectional(end: side1, top: doubleSide2, bottom: doubleSide2)}',
    );
    expect(
      (borderDirectionalWithoutSides + borderWithoutSides).toString(),
      '${const Border(top: doubleSide2, bottom: doubleSide2)}',
    );
  });

  test('BorderDirectional.scale', () {
    const BorderSide magenta3 = BorderSide(color: Color(0xFFFF00FF), width: 3.0);
    const BorderSide magenta6 = BorderSide(color: Color(0xFFFF00FF), width: 6.0);
    const BorderSide yellow2 = BorderSide(color: Color(0xFFFFFF00), width: 2.0);
    const BorderSide yellowNone0 = BorderSide(
      color: Color(0xFFFFFF00),
      width: 0.0,
      style: BorderStyle.none,
    );
    const BorderDirectional b3 = BorderDirectional(start: magenta3);
    const BorderDirectional b6 = BorderDirectional(start: magenta6);
    expect(b3.scale(2.0), b6);
    const BorderDirectional bY0 = BorderDirectional(top: yellowNone0);
    expect(bY0.scale(3.0), bY0);
    const BorderDirectional bY2 = BorderDirectional(top: yellow2);
    expect(bY2.scale(0.0), bY0);
  });

  test('BorderDirectional.lerp', () {
    const BorderDirectional directionalWithTop10 = BorderDirectional(top: BorderSide(width: 10.0));
    const BorderDirectional atMinus100 = BorderDirectional(
      start: BorderSide(width: 0.0),
      end: BorderSide(width: 300.0),
    );
    const BorderDirectional at0 = BorderDirectional(
      start: BorderSide(width: 100.0),
      end: BorderSide(width: 200.0),
    );
    const BorderDirectional at25 = BorderDirectional(
      start: BorderSide(width: 125.0),
      end: BorderSide(width: 175.0),
    );
    const BorderDirectional at75 = BorderDirectional(
      start: BorderSide(width: 175.0),
      end: BorderSide(width: 125.0),
    );
    const BorderDirectional at100 = BorderDirectional(
      start: BorderSide(width: 200.0),
      end: BorderSide(width: 100.0),
    );
    const BorderDirectional at200 = BorderDirectional(
      start: BorderSide(width: 300.0),
      end: BorderSide(width: 0.0),
    );

    expect(BorderDirectional.lerp(null, null, -1.0), null);
    expect(
      BorderDirectional.lerp(directionalWithTop10, null, -1.0),
      const BorderDirectional(top: BorderSide(width: 20.0)),
    );
    expect(BorderDirectional.lerp(null, directionalWithTop10, -1.0), const BorderDirectional());
    expect(BorderDirectional.lerp(at0, at100, -1.0), atMinus100);

    expect(BorderDirectional.lerp(null, null, 0.0), null);
    expect(
      BorderDirectional.lerp(directionalWithTop10, null, 0.0),
      const BorderDirectional(top: BorderSide(width: 10.0)),
    );
    expect(BorderDirectional.lerp(null, directionalWithTop10, 0.0), const BorderDirectional());
    expect(BorderDirectional.lerp(at0, at100, 0.0), at0);

    expect(BorderDirectional.lerp(null, null, 0.25), null);
    expect(
      BorderDirectional.lerp(directionalWithTop10, null, 0.25),
      const BorderDirectional(top: BorderSide(width: 7.5)),
    );
    expect(
      BorderDirectional.lerp(null, directionalWithTop10, 0.25),
      const BorderDirectional(top: BorderSide(width: 2.5)),
    );
    expect(BorderDirectional.lerp(at0, at100, 0.25), at25);

    expect(BorderDirectional.lerp(null, null, 0.75), null);
    expect(
      BorderDirectional.lerp(directionalWithTop10, null, 0.75),
      const BorderDirectional(top: BorderSide(width: 2.5)),
    );
    expect(
      BorderDirectional.lerp(null, directionalWithTop10, 0.75),
      const BorderDirectional(top: BorderSide(width: 7.5)),
    );
    expect(BorderDirectional.lerp(at0, at100, 0.75), at75);

    expect(BorderDirectional.lerp(null, null, 1.0), null);
    expect(BorderDirectional.lerp(directionalWithTop10, null, 1.0), const BorderDirectional());
    expect(
      BorderDirectional.lerp(null, directionalWithTop10, 1.0),
      const BorderDirectional(top: BorderSide(width: 10.0)),
    );
    expect(BorderDirectional.lerp(at0, at100, 1.0), at100);

    expect(BorderDirectional.lerp(null, null, 2.0), null);
    expect(BorderDirectional.lerp(directionalWithTop10, null, 2.0), const BorderDirectional());
    expect(
      BorderDirectional.lerp(null, directionalWithTop10, 2.0),
      const BorderDirectional(top: BorderSide(width: 20.0)),
    );
    expect(BorderDirectional.lerp(at0, at100, 2.0), at200);
  });

  test('BorderDirectional.paint', () {
    expect(
      (Canvas canvas) {
        const BorderDirectional(end: BorderSide(width: 10.0, color: Color(0xFF00FF00))).paint(
          canvas,
          const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
          textDirection: TextDirection.rtl,
        );
      },
      paints..path(
        includes: <Offset>[const Offset(15.0, 30.0)],
        excludes: <Offset>[const Offset(25.0, 30.0)],
        color: const Color(0xFF00FF00),
      ),
    );
    expect(
      (Canvas canvas) {
        const BorderDirectional(end: BorderSide(width: 10.0, color: Color(0xFF00FF00))).paint(
          canvas,
          const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
          textDirection: TextDirection.ltr,
        );
      },
      paints..path(
        includes: <Offset>[const Offset(25.0, 30.0)],
        excludes: <Offset>[const Offset(15.0, 30.0)],
        color: const Color(0xFF00FF00),
      ),
    );
    expect(
      (Canvas canvas) {
        const BorderDirectional(
          end: BorderSide(width: 10.0, color: Color(0xFF00FF00)),
        ).paint(canvas, const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0));
      },
      paintsAssertion, // no TextDirection
    );
  });

  test('BorderDirectional hashCode', () {
    final BorderSide side = BorderSide(width: nonconst(2.0));
    expect(BorderDirectional(top: side).hashCode, BorderDirectional(top: side).hashCode);
    expect(BorderDirectional(top: side).hashCode, isNot(BorderDirectional(bottom: side).hashCode));
  });

  test('BoxDecoration.border takes a BorderDirectional', () {
    const BoxDecoration decoration2 = BoxDecoration(
      border: BorderDirectional(start: BorderSide(width: 2.0)),
    );
    const BoxDecoration decoration4 = BoxDecoration(
      border: BorderDirectional(start: BorderSide(width: 4.0)),
    );
    const BoxDecoration decoration6 = BoxDecoration(
      border: BorderDirectional(start: BorderSide(width: 6.0)),
    );
    final BoxPainter painter = decoration2.createBoxPainter();
    expect(
      (Canvas canvas) {
        painter.paint(
          canvas,
          const Offset(30.0, 0.0),
          const ImageConfiguration(size: Size(20.0, 20.0), textDirection: TextDirection.rtl),
        );
      },
      paints..path(
        includes: <Offset>[const Offset(49.0, 10.0)],
        excludes: <Offset>[const Offset(31.0, 10.0)],
      ),
    );
    expect(
      (Canvas canvas) {
        painter.paint(
          canvas,
          const Offset(30.0, 0.0),
          const ImageConfiguration(size: Size(20.0, 20.0), textDirection: TextDirection.ltr),
        );
      },
      paints..path(
        includes: <Offset>[const Offset(31.0, 10.0)],
        excludes: <Offset>[const Offset(49.0, 10.0)],
      ),
    );
    expect(decoration2.padding, const EdgeInsetsDirectional.fromSTEB(2.0, 0.0, 0.0, 0.0));
    expect(decoration2.scale(2.0), decoration4);
    expect(BoxDecoration.lerp(decoration2, decoration6, 0.5), decoration4);
  });
}
