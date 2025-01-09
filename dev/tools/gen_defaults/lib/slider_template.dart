// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class SliderTemplate extends TokenTemplate {
  const SliderTemplate(
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
  Color? get secondaryActiveTrackColor => ${componentColor('$tokenGroup.active.track')}.withOpacity(0.54);

  @override
  Color? get disabledActiveTrackColor => ${componentColor('$tokenGroup.disabled.active.track')};

  @override
  Color? get disabledInactiveTrackColor => ${componentColor('$tokenGroup.disabled.inactive.track')};

  @override
  Color? get disabledSecondaryActiveTrackColor => ${componentColor('$tokenGroup.disabled.active.track')};

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
  Color? get disabledThumbColor => ${componentColor('$tokenGroup.disabled.handle')};

  @override
  Color? get overlayColor => MaterialStateColor.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.dragged)) {
      return _colors.primary.withOpacity(0.1);
    }
    if (states.contains(MaterialState.hovered)) {
      return _colors.primary.withOpacity(0.08);
    }
    if (states.contains(MaterialState.focused)) {
      return _colors.primary.withOpacity(0.1);
    }

    return Colors.transparent;
  });

  @override
  TextStyle? get valueIndicatorTextStyle => ${textStyle('$tokenGroup.value-indicator.label.label-text')}!.copyWith(
    color: ${componentColor('$tokenGroup.value-indicator.label.label-text')},
  );

  @override
  Color? get valueIndicatorColor => ${componentColor('$tokenGroup.value-indicator.container')};

  @override
  SliderComponentShape? get valueIndicatorShape => const RoundedRectSliderValueIndicatorShape();

  @override
  SliderComponentShape? get thumbShape => const HandleThumbShape();

  @override
  SliderTrackShape? get trackShape => const GappedSliderTrackShape();

  @override
  SliderComponentShape? get overlayShape => const RoundSliderOverlayShape();

  @override
  SliderTickMarkShape? get tickMarkShape => const RoundSliderTickMarkShape(tickMarkRadius: ${getToken("$tokenGroup.stop-indicator.size")} / 2);

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
