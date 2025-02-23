// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'checkbox.dart';
/// @docImport 'radio.dart';
/// @docImport 'switch.dart';
/// @docImport 'text_theme.dart';
library;

import 'package:flutter/widgets.dart';

import 'button.dart';

/// The minimum dimension of any interactive region according to the iOS Human
/// Interface Guidelines.
///
/// This is used to avoid small regions that are hard for the user to interact
/// with. It applies to both dimensions of a region, so a square of size
/// kMinInteractiveDimension x kMinInteractiveDimension is the smallest
/// acceptable region that should respond to gestures.
///
/// See also:
///
///  * [kMinInteractiveDimension]
///  * <https://developer.apple.com/ios/human-interface-guidelines/visual-design/adaptivity-and-layout/>
const double kMinInteractiveDimensionCupertino = 44.0;

/// The relative values needed to transform a color to it's equivalent focus
/// outline color.
///
/// These are used to draw a focus ring around [CupertinoSwitch],
/// [CupertinoCheckbox], [CupertinoRadio] and [CupertinoButton].
///
/// See also:
///
/// * <https://developer.apple.com/design/human-interface-guidelines/focus-and-selection/>
const double kCupertinoFocusColorOpacity = 0.80,
    kCupertinoFocusColorBrightness = 0.69,
    kCupertinoFocusColorSaturation = 0.835;

/// Opacity values for the background of a [CupertinoButton.tinted].
///
/// See also:
///
/// * <https://developer.apple.com/design/human-interface-guidelines/buttons#iOS-iPadOS>
const double kCupertinoButtonTintedOpacityLight = 0.12, kCupertinoButtonTintedOpacityDark = 0.26;

/// The default value for [IconThemeData.size] of [CupertinoButton.child].
///
/// Set to match the most-frequent size of icons in iOS (matches md/lg).
///
/// Used only when the [CupertinoTextThemeData.actionTextStyle] or [CupertinoTextThemeData.actionSmallTextStyle]
/// has a null [TextStyle.fontSize].
const double kCupertinoButtonDefaultIconSize = 20.0;

/// The padding values for the different [CupertinoButtonSize]s.
///
/// Based on the iOS (17) [Human Interface Guidelines](https://developer.apple.com/design/human-interface-guidelines/buttons#iOS-iPadOS).
const Map<CupertinoButtonSize, EdgeInsetsGeometry> kCupertinoButtonPadding =
    <CupertinoButtonSize, EdgeInsetsGeometry>{
      CupertinoButtonSize.small: EdgeInsets.symmetric(vertical: 6, horizontal: 12),
      CupertinoButtonSize.medium: EdgeInsets.symmetric(vertical: 10, horizontal: 15),
      CupertinoButtonSize.large: EdgeInsets.symmetric(vertical: 16, horizontal: 20),
    };

/// The border radius values for the different [CupertinoButtonSize]s.
///
/// Based on the iOS (17) [Human Interface Guidelines](https://developer.apple.com/design/human-interface-guidelines/buttons#iOS-iPadOS).
final Map<CupertinoButtonSize, BorderRadius> kCupertinoButtonSizeBorderRadius =
    <CupertinoButtonSize, BorderRadius>{
      CupertinoButtonSize.small: BorderRadius.circular(40),
      CupertinoButtonSize.medium: BorderRadius.circular(40),
      CupertinoButtonSize.large: BorderRadius.circular(12),
    };

/// The minimum size of a [CupertinoButton] based on the [CupertinoButtonSize].
///
/// Based on the iOS (17) [Human Interface Guidelines](https://developer.apple.com/design/human-interface-guidelines/buttons#iOS-iPadOS).
const Map<CupertinoButtonSize, double> kCupertinoButtonMinSize = <CupertinoButtonSize, double>{
  CupertinoButtonSize.small: 28,
  CupertinoButtonSize.medium: 32,
  CupertinoButtonSize.large: 44,
};

/// The distance a button needs to be moved after being pressed for its opacity to change.
///
/// The opacity changes when the position moved is this distance away from the button.
/// This variable is effective on mobile platforms. For desktop platforms, a distance of 0 is used.
///
/// This value was obtained through actual testing on an iOS 18.1 simulator.
const double kCupertinoButtonTapMoveSlop = 70.0;
