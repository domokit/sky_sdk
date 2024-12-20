// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('MaterialStateProperty.resolveWith()', () {
    final MaterialStateProperty<MaterialState> value =
        MaterialStateProperty.resolveWith<MaterialState>(
          (Set<MaterialState> states) => states.first,
        );
    expect(value.resolve(<MaterialState>{MaterialState.hovered}), MaterialState.hovered);
    expect(value.resolve(<MaterialState>{MaterialState.focused}), MaterialState.focused);
    expect(value.resolve(<MaterialState>{MaterialState.pressed}), MaterialState.pressed);
    expect(value.resolve(<MaterialState>{MaterialState.dragged}), MaterialState.dragged);
    expect(value.resolve(<MaterialState>{MaterialState.selected}), MaterialState.selected);
    expect(value.resolve(<MaterialState>{MaterialState.disabled}), MaterialState.disabled);
    expect(value.resolve(<MaterialState>{MaterialState.error}), MaterialState.error);
  });

  test('MaterialStateProperty.all()', () {
    final MaterialStateProperty<int> value = MaterialStateProperty.all<int>(123);
    expect(value.resolve(<MaterialState>{MaterialState.hovered}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.focused}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.pressed}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.dragged}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.selected}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.disabled}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.error}), 123);
  });

  test('MaterialStatePropertyAll', () {
    const MaterialStatePropertyAll<int> value = MaterialStatePropertyAll<int>(123);
    expect(value.resolve(<MaterialState>{}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.hovered}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.focused}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.pressed}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.dragged}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.selected}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.disabled}), 123);
    expect(value.resolve(<MaterialState>{MaterialState.error}), 123);
  });

  test('toString formats correctly', () {
    const MaterialStateProperty<Color?> colorProperty = MaterialStatePropertyAll<Color?>(
      Color(0xFFFFFFFF),
    );
    expect(colorProperty.toString(), equals('WidgetStatePropertyAll(${const Color(0xffffffff)})'));

    const MaterialStateProperty<double?> doubleProperty = MaterialStatePropertyAll<double?>(
      33 + 1 / 3,
    );
    expect(doubleProperty.toString(), equals('WidgetStatePropertyAll(33.3)'));
  });

  test("Can interpolate between two MaterialStateProperty's", () {
    const MaterialStateProperty<TextStyle?> textStyle1 = MaterialStatePropertyAll<TextStyle?>(
      TextStyle(fontSize: 14.0),
    );
    const MaterialStateProperty<TextStyle?> textStyle2 = MaterialStatePropertyAll<TextStyle?>(
      TextStyle(fontSize: 20.0),
    );

    // Using `0.0` interpolation value.
    TextStyle textStyle =
        MaterialStateProperty.lerp<TextStyle?>(
          textStyle1,
          textStyle2,
          0.0,
          TextStyle.lerp,
        )!.resolve(enabled)!;
    expect(textStyle.fontSize, 14.0);

    // Using `0.5` interpolation value.
    textStyle =
        MaterialStateProperty.lerp<TextStyle?>(
          textStyle1,
          textStyle2,
          0.5,
          TextStyle.lerp,
        )!.resolve(enabled)!;
    expect(textStyle.fontSize, 17.0);

    // Using `1.0` interpolation value.
    textStyle =
        MaterialStateProperty.lerp<TextStyle?>(
          textStyle1,
          textStyle2,
          1.0,
          TextStyle.lerp,
        )!.resolve(enabled)!;
    expect(textStyle.fontSize, 20.0);
  });
}

Set<MaterialState> enabled = <MaterialState>{};
