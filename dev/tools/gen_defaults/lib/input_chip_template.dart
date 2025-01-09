// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class InputChipTemplate extends TokenTemplate {
  const InputChipTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  static const String tokenGroup = 'md.comp.input-chip';
  static const String variant = '';

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends ChipThemeData {
  _${blockName}DefaultsM3(this.context, this.isEnabled, this.isSelected)
    : super(
        elevation: ${elevation("$tokenGroup$variant.container")},
        shape: ${shape("$tokenGroup.container")},
        showCheckmark: true,
      );

  final BuildContext context;
  final bool isEnabled;
  final bool isSelected;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  TextStyle? get labelStyle => ${textStyle("$tokenGroup.label-text")}?.copyWith(
    color: isEnabled
      ? isSelected
        ? ${color("$tokenGroup.selected.label-text.color")}
        : ${color("$tokenGroup.unselected.label-text.color")}
      : ${color("$tokenGroup.disabled.label-text.color")},
  );

  @override
  MaterialStateProperty<Color?>? get color =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected) && states.contains(MaterialState.disabled)) {
        return ${componentColor("$tokenGroup$variant.disabled.selected.container")};
      }
      if (states.contains(MaterialState.disabled)) {
        return ${componentColor("$tokenGroup$variant.disabled.container")};
      }
      if (states.contains(MaterialState.selected)) {
        return ${componentColor("$tokenGroup$variant.selected.container")};
      }
      return ${componentColor("$tokenGroup$variant.container")};
    });

  @override
  Color? get shadowColor => ${colorOrTransparent("$tokenGroup.container.shadow-color")};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent("$tokenGroup.container.surface-tint-layer.color")};

  @override
  Color? get checkmarkColor => isEnabled
    ? isSelected
      ? ${color("$tokenGroup.with-leading-icon.selected.leading-icon.color")}
      : ${color("$tokenGroup.with-leading-icon.unselected.leading-icon.color")}
    : ${color("$tokenGroup.with-leading-icon.disabled.leading-icon.color")};

  @override
  Color? get deleteIconColor => isEnabled
    ? isSelected
      ? ${color("$tokenGroup.with-trailing-icon.selected.trailing-icon.color")}
      : ${color("$tokenGroup.with-trailing-icon.unselected.trailing-icon.color")}
    : ${color("$tokenGroup.with-trailing-icon.disabled.trailing-icon.color")};

  @override
  BorderSide? get side => !isSelected
    ? isEnabled
      ? ${border('$tokenGroup$variant.unselected.outline')}
      : ${border('$tokenGroup$variant.disabled.unselected.outline')}
    : const BorderSide(color: Colors.transparent);

  @override
  IconThemeData? get iconTheme => IconThemeData(
    color: isEnabled
      ? isSelected
        ? ${color("$tokenGroup.with-leading-icon.selected.leading-icon.color")}
        : ${color("$tokenGroup.with-leading-icon.unselected.leading-icon.color")}
      : ${color("$tokenGroup.with-leading-icon.disabled.leading-icon.color")},
    size: ${getToken("$tokenGroup.with-leading-icon.leading-icon.size")},
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
