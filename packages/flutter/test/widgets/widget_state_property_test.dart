// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('WidgetStateProperty.resolveWith()', () {
    final WidgetStateProperty<WidgetState> value = WidgetStateProperty.resolveWith<WidgetState>(
      (Set<WidgetState> states) => states.first,
    );
    expect(value.resolve(<WidgetState>{WidgetState.hovered}), WidgetState.hovered);
    expect(value.resolve(<WidgetState>{WidgetState.focused}), WidgetState.focused);
    expect(value.resolve(<WidgetState>{WidgetState.pressed}), WidgetState.pressed);
    expect(value.resolve(<WidgetState>{WidgetState.dragged}), WidgetState.dragged);
    expect(value.resolve(<WidgetState>{WidgetState.selected}), WidgetState.selected);
    expect(value.resolve(<WidgetState>{WidgetState.disabled}), WidgetState.disabled);
    expect(value.resolve(<WidgetState>{WidgetState.error}), WidgetState.error);
  });

  test('WidgetStateProperty.map()', () {
    final WidgetStatesConstraint active =
        WidgetState.hovered | WidgetState.focused | WidgetState.pressed;
    final WidgetStateProperty<String?> value =
        WidgetStateProperty<String?>.fromMap(<WidgetStatesConstraint, String?>{
          active & WidgetState.error: 'active error',
          WidgetState.disabled | WidgetState.error: 'kinda sus',
          ~(WidgetState.dragged | WidgetState.selected) & ~active: 'this is boring',
          active: 'active',
        });
    expect(value.resolve(<WidgetState>{WidgetState.focused, WidgetState.error}), 'active error');
    expect(value.resolve(<WidgetState>{WidgetState.scrolledUnder}), 'this is boring');
    expect(value.resolve(<WidgetState>{WidgetState.disabled}), 'kinda sus');
    expect(value.resolve(<WidgetState>{WidgetState.hovered}), 'active');
    expect(value.resolve(<WidgetState>{WidgetState.dragged}), null);
  });

  test('WidgetStateProperty.all()', () {
    final WidgetStateProperty<int> value = WidgetStateProperty.all<int>(123);
    expect(value.resolve(<WidgetState>{WidgetState.hovered}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.focused}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.pressed}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.dragged}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.selected}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.disabled}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.error}), 123);
  });

  test('WidgetStatePropertyAll', () {
    const WidgetStatePropertyAll<int> value = WidgetStatePropertyAll<int>(123);
    expect(value.resolve(<WidgetState>{}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.hovered}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.focused}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.pressed}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.dragged}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.selected}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.disabled}), 123);
    expect(value.resolve(<WidgetState>{WidgetState.error}), 123);
  });

  test('toString formats correctly', () {
    const WidgetStateProperty<Color?> colorProperty = WidgetStatePropertyAll<Color?>(
      Color(0xFFFFFFFF),
    );
    expect(colorProperty.toString(), equals('WidgetStatePropertyAll(${const Color(0xffffffff)})'));

    const WidgetStateProperty<double?> doubleProperty = WidgetStatePropertyAll<double?>(33 + 1 / 3);
    expect(doubleProperty.toString(), equals('WidgetStatePropertyAll(33.3)'));
  });

  test("Can interpolate between two WidgetStateProperty's", () {
    const WidgetStateProperty<TextStyle?> textStyle1 = WidgetStatePropertyAll<TextStyle?>(
      TextStyle(fontSize: 14.0),
    );
    const WidgetStateProperty<TextStyle?> textStyle2 = WidgetStatePropertyAll<TextStyle?>(
      TextStyle(fontSize: 20.0),
    );

    // Using `0.0` interpolation value.
    TextStyle textStyle =
        WidgetStateProperty.lerp<TextStyle?>(
          textStyle1,
          textStyle2,
          0.0,
          TextStyle.lerp,
        )!.resolve(enabled)!;
    expect(textStyle.fontSize, 14.0);

    // Using `0.5` interpolation value.
    textStyle =
        WidgetStateProperty.lerp<TextStyle?>(
          textStyle1,
          textStyle2,
          0.5,
          TextStyle.lerp,
        )!.resolve(enabled)!;
    expect(textStyle.fontSize, 17.0);

    // Using `1.0` interpolation value.
    textStyle =
        WidgetStateProperty.lerp<TextStyle?>(
          textStyle1,
          textStyle2,
          1.0,
          TextStyle.lerp,
        )!.resolve(enabled)!;
    expect(textStyle.fontSize, 20.0);
  });

  test('WidgetStateBorderSide.lerp()', () {
    const WidgetStateProperty<BorderSide?> borderSide1 = WidgetStatePropertyAll<BorderSide?>(
      BorderSide(color: Color(0xffff0000), width: 4.0),
    );
    const WidgetStateProperty<BorderSide?> borderSide2 = WidgetStatePropertyAll<BorderSide?>(
      BorderSide(color: Color(0xff0000ff), width: 12.0),
    );

    // Using `0.0` interpolation value.
    BorderSide borderSide =
        WidgetStateBorderSide.lerp(borderSide1, borderSide2, 0.0)!.resolve(enabled)!;
    expect(borderSide.color, isSameColorAs(const Color(0xffff0000)));
    expect(borderSide.width, 4.0);

    // Using `0.5` interpolation value.
    borderSide = WidgetStateBorderSide.lerp(borderSide1, borderSide2, 0.5)!.resolve(enabled)!;
    expect(borderSide.color, isSameColorAs(const Color(0xff7f007f)));
    expect(borderSide.width, 8.0);

    // Using `1.0` interpolation value.
    borderSide = WidgetStateBorderSide.lerp(borderSide1, borderSide2, 1.0)!.resolve(enabled)!;
    expect(borderSide.color, isSameColorAs(const Color(0xff0000ff)));
    expect(borderSide.width, 12.0);
  });

  test('.fromMap() constructors perform accurate equality checks', () {
    const Color white = Color(0xFFFFFFFF);
    const Color black = Color(0xFF000000);
    final WidgetStateColor color1 = WidgetStateColor.fromMap(<WidgetStatesConstraint, Color>{
      WidgetState.focused | WidgetState.hovered: white,
      WidgetState.any: black,
    });
    final WidgetStateColor color2 = WidgetStateColor.fromMap(<WidgetStatesConstraint, Color>{
      WidgetState.focused | WidgetState.hovered: white,
      WidgetState.any: black,
    });
    final WidgetStateColor color3 = WidgetStateColor.fromMap(<WidgetStatesConstraint, Color>{
      WidgetState.focused | WidgetState.hovered: black,
      WidgetState.any: white,
    });
    expect(color1 == color2, isTrue);
    expect(color1 == color3, isFalse);

    const BorderSide whiteBorder = BorderSide(color: white);
    const BorderSide blackBorder = BorderSide();
    final WidgetStateBorderSide side1 = WidgetStateBorderSide.fromMap(
      <WidgetStatesConstraint, BorderSide>{
        WidgetState.focused | WidgetState.hovered: whiteBorder,
        WidgetState.any: blackBorder,
      },
    );
    final WidgetStateBorderSide side2 = WidgetStateBorderSide.fromMap(
      <WidgetStatesConstraint, BorderSide>{
        WidgetState.focused | WidgetState.hovered: whiteBorder,
        WidgetState.any: blackBorder,
      },
    );
    final WidgetStateBorderSide side3 = WidgetStateBorderSide.fromMap(
      <WidgetStatesConstraint, BorderSide>{
        WidgetState.focused | WidgetState.hovered: blackBorder,
        WidgetState.any: whiteBorder,
      },
    );
    expect(side1 == side2, isTrue);
    expect(side1 == side3, isFalse);

    const OutlinedBorder whiteRRect = RoundedRectangleBorder(side: whiteBorder);
    const OutlinedBorder blackRRect = RoundedRectangleBorder(side: blackBorder);
    final WidgetStateOutlinedBorder border1 = WidgetStateOutlinedBorder.fromMap(
      <WidgetStatesConstraint, OutlinedBorder>{
        WidgetState.focused | WidgetState.hovered: whiteRRect,
        WidgetState.any: blackRRect,
      },
    );
    final WidgetStateOutlinedBorder border2 = WidgetStateOutlinedBorder.fromMap(
      <WidgetStatesConstraint, OutlinedBorder>{
        WidgetState.focused | WidgetState.hovered: whiteRRect,
        WidgetState.any: blackRRect,
      },
    );
    final WidgetStateOutlinedBorder border3 = WidgetStateOutlinedBorder.fromMap(
      <WidgetStatesConstraint, OutlinedBorder>{
        WidgetState.focused | WidgetState.hovered: blackRRect,
        WidgetState.any: whiteRRect,
      },
    );
    expect(border1 == border2, isTrue);
    expect(border1 == border3, isFalse);

    final WidgetStateMouseCursor cursor1 =
        WidgetStateMouseCursor.fromMap(<WidgetStatesConstraint, MouseCursor>{
          WidgetState.focused | WidgetState.hovered: MouseCursor.defer,
          WidgetState.any: MouseCursor.uncontrolled,
        });
    final WidgetStateMouseCursor cursor2 =
        WidgetStateMouseCursor.fromMap(<WidgetStatesConstraint, MouseCursor>{
          WidgetState.focused | WidgetState.hovered: MouseCursor.defer,
          WidgetState.any: MouseCursor.uncontrolled,
        });
    final WidgetStateMouseCursor cursor3 =
        WidgetStateMouseCursor.fromMap(<WidgetStatesConstraint, MouseCursor>{
          WidgetState.focused | WidgetState.hovered: MouseCursor.uncontrolled,
          WidgetState.any: MouseCursor.defer,
        });
    expect(cursor1 == cursor2, isTrue);
    expect(cursor1 == cursor3, isFalse);

    const TextStyle whiteText = TextStyle(color: white);
    const TextStyle blackText = TextStyle(color: black);
    final WidgetStateTextStyle style1 = WidgetStateTextStyle.fromMap(
      <WidgetStatesConstraint, TextStyle>{
        WidgetState.focused | WidgetState.hovered: whiteText,
        WidgetState.any: blackText,
      },
    );
    final WidgetStateTextStyle style2 = WidgetStateTextStyle.fromMap(
      <WidgetStatesConstraint, TextStyle>{
        WidgetState.focused | WidgetState.hovered: whiteText,
        WidgetState.any: blackText,
      },
    );
    final WidgetStateTextStyle style3 = WidgetStateTextStyle.fromMap(
      <WidgetStatesConstraint, TextStyle>{
        WidgetState.focused | WidgetState.hovered: blackText,
        WidgetState.any: whiteText,
      },
    );
    expect(style1 == style2, isTrue);
    expect(style1 == style3, isFalse);
  });
}

const Set<WidgetState> enabled = <WidgetState>{};
