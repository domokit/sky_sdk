// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class RangeSliderTemplate extends TokenTemplate {
  const RangeSliderTemplate(
    this.tokenGroup,
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  final String tokenGroup;

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends SliderThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(trackHeight: ${getToken('$tokenGroup.active.track.height')});

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  Color? get activeTrackColor => ${componentColor('$tokenGroup.active.track')};

  @override
  Color? get inactiveTrackColor => ${componentColor('$tokenGroup.inactive.track')};

  @override
  Color? get disabledActiveTrackColor => ${componentColor('$tokenGroup.disabled.active.track')};

  @override
  Color? get disabledInactiveTrackColor => ${componentColor('$tokenGroup.disabled.inactive.track')};

  @override
  Color? get activeTickMarkColor => ${componentColor('$tokenGroup.active.stop-indicator.container')};

  @override
  Color? get inactiveTickMarkColor => ${componentColor('$tokenGroup.inactive.stop-indicator.container')};

  @override
  Color? get disabledActiveTickMarkColor => ${componentColor('$tokenGroup.disabled.active.stop-indicator.container')};

  @override
  Color? get disabledInactiveTickMarkColor => ${componentColor('$tokenGroup.disabled.inactive.stop-indicator.container')};

  @override
  Color? get thumbColor => ${componentColor('$tokenGroup.handle')};

  @override
  ui.Color? get overlappingShapeStrokeColor => _colors.surface;

  @override
  Color? get disabledThumbColor => ${componentColor('$tokenGroup.disabled.handle')};

  @override
  Color? get overlayColor => _colors.primary.withOpacity(0.12);

  @override
  TextStyle? get valueIndicatorTextStyle => ${textStyle('$tokenGroup.value-indicator.label.label-text')}!.copyWith(
    color: ${componentColor('$tokenGroup.value-indicator.label.label-text')},
  );

  @override
  Color? get valueIndicatorColor => ${componentColor('$tokenGroup.value-indicator.container')};

  @override
  RangeSliderTrackShape? get rangeTrackShape => const GappedRangeSliderTrackShape();

  @override
  RangeSliderTickMarkShape? get rangeTickMarkShape => const RoundRangeSliderTickMarkShape(tickMarkRadius: ${getToken("$tokenGroup.stop-indicator.size")} / 2);

  @override
  RangeSliderThumbShape? get rangeThumbShape => const HandleRangeSliderThumbShape();

  @override
  SliderComponentShape? get overlayShape => const RoundSliderOverlayShape();

  @override
  RangeSliderValueIndicatorShape? get rangeValueIndicatorShape => const RoundedRectRangeSliderValueIndicatorShape();

  @override
  ShowValueIndicator? get showValueIndicator => ShowValueIndicator.onlyForDiscrete;

  @override
  double? get minThumbSeparation => 0;

  @override
  MaterialStateProperty<Size?>? get thumbSize {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return const Size(${getToken("$tokenGroup.disabled.handle.width")}, ${getToken("$tokenGroup.handle.height")});
      }
      if (states.contains(MaterialState.hovered)) {
        return const Size(${getToken("$tokenGroup.hover.handle.width")}, ${getToken("$tokenGroup.handle.height")});
      }
      if (states.contains(MaterialState.focused)) {
        return const Size(${getToken("$tokenGroup.focus.handle.width")}, ${getToken("$tokenGroup.handle.height")});
      }
      if (states.contains(MaterialState.pressed)) {
        return const Size(${getToken("$tokenGroup.pressed.handle.width")}, ${getToken("$tokenGroup.handle.height")});
      }
      return const Size(${getToken("$tokenGroup.handle.width")}, ${getToken("$tokenGroup.handle.height")});
    });
  }

  @override
  double? get trackGap => ${getToken("$tokenGroup.active.handle.padding")};
}
''';
}
