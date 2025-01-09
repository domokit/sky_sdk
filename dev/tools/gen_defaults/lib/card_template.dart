// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class CardTemplate extends TokenTemplate {
  const CardTemplate(
    this.tokenGroup,
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  final String tokenGroup;

  String _shape() {
    final String cardShape = shape('$tokenGroup.container');
    if (tokenAvailable('$tokenGroup.outline.color')) {
      return '''

    $cardShape.copyWith(
      side: ${border('$tokenGroup.outline')}
    )''';
    } else {
      return cardShape;
    }
  }

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends CardThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(
        clipBehavior: Clip.none,
        elevation: ${elevation('$tokenGroup.container')},
        margin: const EdgeInsets.all(4.0),
      );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  Color? get color => ${componentColor('$tokenGroup.container')};

  @override
  Color? get shadowColor => ${colorOrTransparent('$tokenGroup.container.shadow-color')};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent('$tokenGroup.container.surface-tint-layer.color')};

  @override
  ShapeBorder? get shape =>${_shape()};
}
''';
}
