// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class ActionChipTemplate extends TokenTemplate {
  const ActionChipTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  static const String tokenGroup = 'md.comp.assist-chip';
  static const String flatVariant = '.flat';
  static const String elevatedVariant = '.elevated';

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends ChipThemeData {
  _${blockName}DefaultsM3(this.context, this.isEnabled, this._chipVariant)
    : super(
        shape: ${shape("$tokenGroup.container")},
        showCheckmark: true,
      );

  final BuildContext context;
  final bool isEnabled;
  final _ChipVariant _chipVariant;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  double? get elevation => _chipVariant == _ChipVariant.flat
    ? ${elevation("$tokenGroup$flatVariant.container")}
    : isEnabled ? ${elevation("$tokenGroup$elevatedVariant.container")} : ${elevation("$tokenGroup$elevatedVariant.disabled.container")};

  @override
  double? get pressElevation => ${elevation("$tokenGroup$elevatedVariant.pressed.container")};

  @override
  TextStyle? get labelStyle => ${textStyle("$tokenGroup.label-text")}?.copyWith(
    color: isEnabled
      ? ${color("$tokenGroup.label-text.color")}
      : ${color("$tokenGroup.disabled.label-text.color")},
  );

  @override
  MaterialStateProperty<Color?>? get color =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _chipVariant == _ChipVariant.flat
          ? ${componentColor("$tokenGroup$flatVariant.disabled.container")}
          : ${componentColor("$tokenGroup$elevatedVariant.disabled.container")};
      }
      return _chipVariant == _ChipVariant.flat
        ? ${componentColor("$tokenGroup$flatVariant.container")}
        : ${componentColor("$tokenGroup$elevatedVariant.container")};
    });

  @override
  Color? get shadowColor => _chipVariant == _ChipVariant.flat
    ? ${colorOrTransparent("$tokenGroup$flatVariant.container.shadow-color")}
    : ${colorOrTransparent("$tokenGroup$elevatedVariant.container.shadow-color")};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent("$tokenGroup.container.surface-tint-layer.color")};

  @override
  Color? get checkmarkColor => null;

  @override
  Color? get deleteIconColor => null;

  @override
  BorderSide? get side => _chipVariant == _ChipVariant.flat
    ? isEnabled
        ? ${border('$tokenGroup$flatVariant.outline')}
        : ${border('$tokenGroup$flatVariant.disabled.outline')}
    : const BorderSide(color: Colors.transparent);

  @override
  IconThemeData? get iconTheme => IconThemeData(
    color: isEnabled
      ? ${color("$tokenGroup.with-icon.icon.color")}
      : ${color("$tokenGroup.with-icon.disabled.icon.color")},
    size: ${getToken("$tokenGroup.with-icon.icon.size")},
  );

  @override
  EdgeInsetsGeometry? get padding => const EdgeInsets.all(8.0);

  /// The label padding of the chip scales with the font size specified in the
  /// [labelStyle], and the system font size settings that scale font sizes
  /// globally.
  ///
  /// The chip at effective font size 14.0 starts with 8px on each side and as
  /// the font size scales up to closer to 28.0, the label padding is linearly
  /// interpolated from 8px to 4px. Once the label has a font size of 2 or
  /// higher, label padding remains 4px.
  @override
  EdgeInsetsGeometry? get labelPadding {
    final double fontSize = labelStyle?.fontSize ?? 14.0;
    final double fontSizeRatio = MediaQuery.textScalerOf(context).scale(fontSize) / 14.0;
    return EdgeInsets.lerp(
      const EdgeInsets.symmetric(horizontal: 8.0),
      const EdgeInsets.symmetric(horizontal: 4.0),
      clampDouble(fontSizeRatio - 1.0, 0.0, 1.0),
    )!;
  }
}
''';
}
