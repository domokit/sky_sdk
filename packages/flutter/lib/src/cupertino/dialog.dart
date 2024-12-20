// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'button.dart';
/// @docImport 'route.dart';
library;

import 'dart:math' as math;
import 'dart:ui' show ImageFilter, lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'interface_level.dart';
import 'localizations.dart';
import 'scrollbar.dart';
import 'theme.dart';

// TODO(abarth): These constants probably belong somewhere more general.

// Used XD to flutter plugin(https://github.com/AdobeXD/xd-to-flutter-plugin/)
// to derive values of TextStyle(height and letterSpacing) from
// Adobe XD template for iOS 13, which can be found in
// Apple Design Resources(https://developer.apple.com/design/resources/).
// However the values are not exactly the same as native, so eyeballing is needed.
const TextStyle _kCupertinoDialogTitleStyle = TextStyle(
  fontFamily: 'CupertinoSystemText',
  inherit: false,
  fontSize: 17.0,
  fontWeight: FontWeight.w600,
  height: 1.3,
  letterSpacing: -0.5,
  textBaseline: TextBaseline.alphabetic,
);

const TextStyle _kCupertinoDialogContentStyle = TextStyle(
  fontFamily: 'CupertinoSystemText',
  inherit: false,
  fontSize: 13.0,
  fontWeight: FontWeight.w400,
  height: 1.35,
  letterSpacing: -0.2,
  textBaseline: TextBaseline.alphabetic,
);

const TextStyle _kCupertinoDialogActionStyle = TextStyle(
  fontFamily: 'CupertinoSystemText',
  inherit: false,
  fontSize: 16.8,
  fontWeight: FontWeight.w400,
  textBaseline: TextBaseline.alphabetic,
);

// CupertinoActionSheet-specific text styles.
const TextStyle _kActionSheetActionStyle = TextStyle(
  // The fontSize and fontWeight may be adjusted when the text is rendered.
  fontFamily: 'CupertinoSystemDisplay',
  inherit: false,
  fontSize: 17.0,
  fontWeight: FontWeight.w400,
  textBaseline: TextBaseline.alphabetic,
);

const TextStyle _kActionSheetContentStyle = TextStyle(
  fontFamily: 'CupertinoSystemText',
  inherit: false,
  fontSize: 13.0,
  fontWeight: FontWeight.w400,
  textBaseline: TextBaseline.alphabetic,
  // The `color` is configured by _kActionSheetContentTextColor to be dynamic on
  // context.
);

// Generic constants shared between Dialog and ActionSheet.
const double _kCornerRadius = 14.0;
const double _kDividerThickness = 0.3;

// Dialog specific constants.
// iOS dialogs have a normal display width and another display width that is
// used when the device is in accessibility mode. Each of these widths are
// listed below.
const double _kCupertinoDialogWidth = 270.0;
const double _kAccessibilityCupertinoDialogWidth = 310.0;
const double _kDialogEdgePadding = 20.0;
const double _kDialogMinButtonHeight = 45.0;
const double _kDialogMinButtonFontSize = 10.0;
// The min height for a button excluding dividers. Derived by comparing on iOS
// 17 simulators.
const double _kDialogActionsSectionMinHeight = 67.8;

// ActionSheet specific constants.
const double _kActionSheetEdgePadding = 8.0;
const double _kActionSheetCancelButtonPadding = 8.0;
const double _kActionSheetContentHorizontalPadding = 16.0;
const double _kActionSheetContentVerticalPadding = 13.5;
const double _kActionSheetActionsSectionMinHeight = 84.0;
const double _kActionSheetButtonHorizontalPadding = 10.0;

// According to experimenting on the simulator, the height of action sheet
// buttons is proportional to the font size down to a minimal height.
const double _kActionSheetButtonMinHeight = 57.17;
const double _kActionSheetButtonVerticalPaddingFactor = 0.4;
const double _kActionSheetButtonVerticalPaddingBase = 1.8;

// A translucent color that is painted on top of the blurred backdrop as the
// dialog's background color
// Extracted from https://developer.apple.com/design/resources/.
const Color _kDialogColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xCCF2F2F2),
  darkColor: Color(0xCC2D2D2D),
);

// Translucent light gray that is painted on top of the blurred backdrop as the
// background color of a pressed button.
// Eyeballed from iOS 13 beta simulator.
const Color _kDialogPressedColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xFFE1E1E1),
  darkColor: Color(0xFF404040),
);

// Translucent light gray that is painted on top of the blurred backdrop as the
// background color of a pressed button.
// Eyeballed from iOS 17 simulator.
const Color _kActionSheetPressedColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xCAE0E0E0),
  darkColor: Color(0xC1515151),
);

const Color _kActionSheetCancelColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xFFFFFFFF),
  darkColor: Color(0xFF2C2C2C),
);
const Color _kActionSheetCancelPressedColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xFFECECEC),
  darkColor: Color(0xFF494949),
);

// Translucent, very light gray that is painted on top of the blurred backdrop
// as the action sheet's background color.
// TODO(LongCatIsLooong): https://github.com/flutter/flutter/issues/39272. Use
// System Materials once we have them.
// Eyeballed from iOS 17 simulator.
const Color _kActionSheetBackgroundColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xC8FCFCFC),
  darkColor: Color(0xBE292929),
);

// The gray color used for text that appears in the title area.
// Eyeballed from iOS 17 simulator.
const Color _kActionSheetContentTextColor = CupertinoDynamicColor.withBrightness(
  color: Color(0x851D1D1D),
  darkColor: Color(0x96F1F1F1),
);

// Translucent gray that is painted on top of the blurred backdrop in the gap
// areas between the content section and actions section, as well as between
// buttons.
// Eyeballed from iOS 17 simulator.
const Color _kActionSheetButtonDividerColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xD4C9C9C9),
  darkColor: Color(0xD57D7D7D),
);

// The alert dialog layout policy changes depending on whether the user is using
// a "regular" font size vs a "large" font size. This is a spectrum. There are
// many "regular" font sizes and many "large" font sizes. But depending on which
// policy is currently being used, a dialog is laid out differently.
//
// Empirically, the jump from one policy to the other occurs at the following text
// scale factors:
// Largest regular scale factor:  1.3529411764705883
// Smallest large scale factor:   1.6470588235294117
//
// The following constant represents a division in text scale factor beyond which
// we want to change how the dialog is laid out.
const double _kMaxRegularTextScaleFactor = 1.4;

// Accessibility mode on iOS is determined by the text scale factor that the
// user has selected.
bool _isInAccessibilityMode(BuildContext context) {
  const double defaultFontSize = 14.0;
  final double? scaledFontSize = MediaQuery.maybeTextScalerOf(context)?.scale(defaultFontSize);
  return scaledFontSize != null && scaledFontSize > defaultFontSize * _kMaxRegularTextScaleFactor;
}

/// An iOS-style alert dialog.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=75CsnyRXf5I}
///
/// An alert dialog informs the user about situations that require
/// acknowledgment. An alert dialog has an optional title, optional content,
/// and an optional list of actions. The title is displayed above the content
/// and the actions are displayed below the content.
///
/// This dialog styles its title and content (typically a message) to match the
/// standard iOS title and message dialog text style. These default styles can
/// be overridden by explicitly defining [TextStyle]s for [Text] widgets that
/// are part of the title or content.
///
/// To display action buttons that look like standard iOS dialog buttons,
/// provide [CupertinoDialogAction]s for the [actions] given to this dialog.
///
/// Typically passed as the child widget to [showDialog], which displays the
/// dialog.
///
/// {@tool dartpad}
/// This sample shows how to use a [CupertinoAlertDialog].
///	The [CupertinoAlertDialog] shows an alert with a set of two choices
/// when [CupertinoButton] is pressed.
///
/// ** See code in examples/api/lib/cupertino/dialog/cupertino_alert_dialog.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoPopupSurface], which is a generic iOS-style popup surface that
///    holds arbitrary content to create custom popups.
///  * [CupertinoDialogAction], which is an iOS-style dialog button.
///  * [AlertDialog], a Material Design alert dialog.
///  * <https://developer.apple.com/design/human-interface-guidelines/alerts/>
class CupertinoAlertDialog extends StatefulWidget {
  /// Creates an iOS-style alert dialog.
  const CupertinoAlertDialog({
    super.key,
    this.title,
    this.content,
    this.actions = const <Widget>[],
    this.scrollController,
    this.actionScrollController,
    this.insetAnimationDuration = const Duration(milliseconds: 100),
    this.insetAnimationCurve = Curves.decelerate,
  });

  /// The (optional) title of the dialog is displayed in a large font at the top
  /// of the dialog.
  ///
  /// Typically a [Text] widget.
  final Widget? title;

  /// The (optional) content of the dialog is displayed in the center of the
  /// dialog in a lighter font.
  ///
  /// Typically a [Text] widget.
  final Widget? content;

  /// The (optional) set of actions that are displayed at the bottom of the
  /// dialog.
  ///
  /// Typically this is a list of [CupertinoDialogAction] widgets.
  final List<Widget> actions;

  /// A scroll controller that can be used to control the scrolling of the
  /// [content] in the dialog.
  ///
  /// Defaults to null, which means the [CupertinoDialogAction] will create a
  /// scroll controller internally.
  ///
  /// See also:
  ///
  ///  * [actionScrollController], which can be used for controlling the actions
  ///    section when there are many actions.
  final ScrollController? scrollController;

  /// A scroll controller that can be used to control the scrolling of the
  /// actions in the dialog.
  ///
  /// Defaults to null, which means the [CupertinoDialogAction] will create an
  /// action scroll controller internally.
  ///
  /// See also:
  ///
  ///  * [scrollController], which can be used for controlling the [content]
  ///    section when it is long.
  final ScrollController? actionScrollController;

  /// {@macro flutter.material.dialog.insetAnimationDuration}
  final Duration insetAnimationDuration;

  /// {@macro flutter.material.dialog.insetAnimationCurve}
  final Curve insetAnimationCurve;

  @override
  State<CupertinoAlertDialog> createState() => _CupertinoAlertDialogState();
}

class _CupertinoAlertDialogState extends State<CupertinoAlertDialog> {
  // The index of the action button that the user is holding on.
  //
  // Null if the user is not holding on any buttons.
  int? _pressedIndex;

  ScrollController? _backupScrollController;

  ScrollController? _backupActionScrollController;

  ScrollController get _effectiveScrollController =>
      widget.scrollController ?? (_backupScrollController ??= ScrollController());

  ScrollController get _effectiveActionScrollController =>
      widget.actionScrollController ?? (_backupActionScrollController ??= ScrollController());

  Widget? _buildContent(BuildContext context) {
    final bool hasContent = widget.title != null || widget.content != null;
    if (!hasContent) {
      return null;
    }

    const double defaultFontSize = 14.0;
    final double effectiveTextScaleFactor =
        MediaQuery.textScalerOf(context).scale(defaultFontSize) / defaultFontSize;

    final Widget child = _CupertinoAlertContentSection(
      title: widget.title,
      message: widget.content,
      scrollController: _effectiveScrollController,
      titlePadding: EdgeInsets.only(
        left: _kDialogEdgePadding,
        right: _kDialogEdgePadding,
        bottom: widget.content == null ? _kDialogEdgePadding : 1.0,
        top: _kDialogEdgePadding * effectiveTextScaleFactor,
      ),
      messagePadding: EdgeInsets.only(
        left: _kDialogEdgePadding,
        right: _kDialogEdgePadding,
        bottom: _kDialogEdgePadding * effectiveTextScaleFactor,
        top: widget.title == null ? _kDialogEdgePadding : 1.0,
      ),
      titleTextStyle: _kCupertinoDialogTitleStyle.copyWith(
        color: CupertinoDynamicColor.resolve(CupertinoColors.label, context),
      ),
      messageTextStyle: _kCupertinoDialogContentStyle.copyWith(
        color: CupertinoDynamicColor.resolve(CupertinoColors.label, context),
      ),
    );

    return ColoredBox(color: CupertinoDynamicColor.resolve(_kDialogColor, context), child: child);
  }

  void _onPressedUpdate(int actionIndex, bool isPressed) {
    if (isPressed) {
      setState(() {
        _pressedIndex = actionIndex;
      });
    } else {
      if (_pressedIndex == actionIndex) {
        setState(() {
          _pressedIndex = null;
        });
      }
    }
  }

  Widget? _buildActions() {
    if (widget.actions.isEmpty) {
      return null;
    } else {
      return _CupertinoAlertActionSection(
        scrollController: _effectiveActionScrollController,
        actions: widget.actions,
        pressedIndex: _pressedIndex,
        onPressedUpdate: _onPressedUpdate,
      );
    }
  }

  Widget _buildBody(BuildContext context) {
    final Color backgroundColor = CupertinoDynamicColor.resolve(_kDialogColor, context);
    const Color dividerColor = CupertinoColors.separator;
    // Remove view padding here because the `Scrollbar` widget uses the view
    // padding as padding, which is unwanted.
    // https://github.com/flutter/flutter/issues/150544
    return MediaQuery.removePadding(
      removeLeft: true,
      removeTop: true,
      removeRight: true,
      removeBottom: true,
      context: context,
      child: LayoutBuilder(
        builder: (BuildContext context, BoxConstraints constraints) {
          final Widget? contentSection = _buildContent(context);
          final Widget? actionsSection = _buildActions();
          if (actionsSection == null) {
            return contentSection ??
                const LimitedBox(maxWidth: 0, child: SizedBox(width: double.infinity, height: 0));
          }
          final Widget scrolledActionsSection = _OverscrollBackground(
            color: backgroundColor,
            child: actionsSection,
          );
          if (contentSection == null) {
            return scrolledActionsSection;
          }
          // It is observed on the simulator that the minimal height varies
          // depending on whether the device is in accessibility mode.
          final double actionsMinHeight =
              _isInAccessibilityMode(context)
                  ? constraints.maxHeight / 2 + _kDividerThickness
                  : _kDialogActionsSectionMinHeight + _kDividerThickness;
          return _PriorityColumn(
            top: contentSection,
            bottom: Column(
              children: <Widget>[
                _Divider(dividerColor: dividerColor, hiddenColor: backgroundColor, hidden: false),
                Flexible(child: scrolledActionsSection),
              ],
            ),
            bottomMinHeight: actionsMinHeight,
          );
        },
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final CupertinoLocalizations localizations = CupertinoLocalizations.of(context);
    final bool isInAccessibilityMode = _isInAccessibilityMode(context);
    return CupertinoUserInterfaceLevel(
      data: CupertinoUserInterfaceLevelData.elevated,
      child: MediaQuery.withClampedTextScaling(
        // iOS does not shrink dialog content below a 1.0 scale factor
        minScaleFactor: 1.0,
        child: ScrollConfiguration(
          // A CupertinoScrollbar is built-in below.
          behavior: ScrollConfiguration.of(context).copyWith(scrollbars: false),
          child: LayoutBuilder(
            builder: (BuildContext context, BoxConstraints constraints) {
              return AnimatedPadding(
                padding:
                    MediaQuery.viewInsetsOf(context) +
                    const EdgeInsets.symmetric(horizontal: 40.0, vertical: 24.0),
                duration: widget.insetAnimationDuration,
                curve: widget.insetAnimationCurve,
                child: MediaQuery.removeViewInsets(
                  removeLeft: true,
                  removeTop: true,
                  removeRight: true,
                  removeBottom: true,
                  context: context,
                  child: Center(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(vertical: _kDialogEdgePadding),
                      child: SizedBox(
                        width:
                            isInAccessibilityMode
                                ? _kAccessibilityCupertinoDialogWidth
                                : _kCupertinoDialogWidth,
                        child: _ActionSheetGestureDetector(
                          child: CupertinoPopupSurface(
                            isSurfacePainted: false,
                            child: Semantics(
                              namesRoute: true,
                              scopesRoute: true,
                              explicitChildNodes: true,
                              label: localizations.alertDialogLabel,
                              child: _buildBody(context),
                            ),
                          ),
                        ),
                      ),
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );
  }

  @override
  void dispose() {
    _backupScrollController?.dispose();
    _backupActionScrollController?.dispose();
    super.dispose();
  }
}

/// An iOS-style component for creating modal overlays like dialogs and action
/// sheets.
///
/// By default, [CupertinoPopupSurface] generates a rounded rectangle surface
/// that applies two effects to the background content:
///
///   1. Background filter: Saturates and then blurs content behind the surface.
///   2. Overlay color: Covers the filtered background with a transparent
///      surface color. The color adapts to the CupertinoTheme's brightness:
///      light gray when the ambient [CupertinoTheme] brightness is
///      [Brightness.light], and dark gray when [Brightness.dark].
///
/// The blur strength can be changed by setting [blurSigma] to a positive value,
/// or removed by setting the [blurSigma] to 0.
///
/// The saturation effect can be removed for debugging by setting
/// [debugIsVibrancePainted] to false. The saturation effect is not supported on
/// web with the skwasm renderer and will not be applied regardless of the value
/// of [debugIsVibrancePainted].
///
/// The surface color can be disabled by setting [isSurfacePainted] to false,
/// which is useful for more complicated layouts, such as rendering divider gaps
/// in [CupertinoAlertDialog] or rendering custom surface colors.
///
/// {@tool dartpad}
/// This sample shows how to use a [CupertinoPopupSurface]. The [CupertinoPopupSurface]
/// shows a modal popup from the bottom of the screen.
/// Toggle the switch to configure its surface color.
///
/// ** See code in examples/api/lib/cupertino/dialog/cupertino_popup_surface.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoAlertDialog], which is a dialog with a title, content, and
///    actions.
///  * <https://developer.apple.com/design/human-interface-guidelines/alerts/>
class CupertinoPopupSurface extends StatelessWidget {
  /// Creates an iOS-style rounded rectangle popup surface.
  const CupertinoPopupSurface({
    super.key,
    this.blurSigma = defaultBlurSigma,
    this.isSurfacePainted = true,
    required this.child,
  }) : assert(blurSigma >= 0, 'CupertinoPopupSurface requires a non-negative blur sigma.');

  /// The strength of the gaussian blur applied to the area beneath this
  /// surface.
  ///
  /// Defaults to [defaultBlurSigma]. Setting [blurSigma] to 0 will remove the
  /// blur filter.
  final double blurSigma;

  /// Whether or not to paint a translucent white on top of this surface's
  /// blurred background. [isSurfacePainted] should be true for a typical popup
  /// that contains content without any dividers. A popup that requires dividers
  /// should set [isSurfacePainted] to false and then paint its own surface area.
  ///
  /// Some popups, like iOS's volume control popup, choose to render a blurred
  /// area without any white paint covering it. To achieve this effect,
  /// [isSurfacePainted] should be set to false.
  ///
  /// Defaults to true.
  final bool isSurfacePainted;

  /// The widget below this widget in the tree.
  // Because [CupertinoPopupSurface] is composed of proxy boxes, which mimic
  // the size of their child, a [child] is required to ensure that this surface
  // has a size.
  final Widget child;

  /// The default strength of the blur applied to widgets underlying a
  /// [CupertinoPopupSurface].
  ///
  /// Eyeballed from the iOS 17 simulator.
  static const double defaultBlurSigma = 30.0;

  /// The default corner radius of a [CupertinoPopupSurface].
  static const BorderRadius _clipper = BorderRadius.all(Radius.circular(14));

  // The [ColorFilter] matrix used to saturate widgets underlying a
  // [CupertinoPopupSurface] when the ambient [CupertinoThemeData.brightness] is
  // [Brightness.light].
  //
  // To derive this matrix, the saturation matrix was taken from
  // https://docs.rainmeter.net/tips/colormatrix-guide/ and was tweaked to
  // resemble the iOS 17 simulator.
  //
  // The matrix can be derived from the following function:
  // static List<double> get _lightSaturationMatrix {
  //    const double lightLumR = 0.26;
  //    const double lightLumG = 0.4;
  //    const double lightLumB = 0.17;
  //    const double saturation = 2.0;
  //    const double sr = (1 - saturation) * lightLumR;
  //    const double sg = (1 - saturation) * lightLumG;
  //    const double sb = (1 - saturation) * lightLumB;
  //    return <double>[
  //      sr + saturation, sg, sb, 0.0, 0.0,
  //      sr, sg + saturation, sb, 0.0, 0.0,
  //      sr, sg, sb + saturation, 0.0, 0.0,
  //      0.0, 0.0, 0.0, 1.0, 0.0,
  //    ];
  //  }
  static const List<double> _lightSaturationMatrix = <double>[
    1.74,
    -0.40,
    -0.17,
    0.00,
    0.00,
    -0.26,
    1.60,
    -0.17,
    0.00,
    0.00,
    -0.26,
    -0.40,
    1.83,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    1.00,
    0.00,
  ];

  // The [ColorFilter] matrix used to saturate widgets underlying a
  // [CupertinoPopupSurface] when the ambient [CupertinoThemeData.brightness] is
  // [Brightness.dark].
  //
  // To derive this matrix, the saturation matrix was taken from
  // https://docs.rainmeter.net/tips/colormatrix-guide/ and was tweaked to
  // resemble the iOS 17 simulator.
  //
  // The matrix can be derived from the following function:
  // static List<double> get _darkSaturationMatrix {
  //    const double additive = 0.3;
  //    const double darkLumR = 0.45;
  //    const double darkLumG = 0.8;
  //    const double darkLumB = 0.16;
  //    const double saturation = 1.7;
  //    const double sr = (1 - saturation) * darkLumR;
  //    const double sg = (1 - saturation) * darkLumG;
  //    const double sb = (1 - saturation) * darkLumB;
  //    return <double>[
  //      sr + saturation, sg, sb, 0.0, additive,
  //      sr, sg + saturation, sb, 0.0, additive,
  //      sr, sg, sb + saturation, 0.0, additive,
  //      0.0, 0.0, 0.0, 1.0, 0.0,
  //    ];
  //  }
  static const List<double> _darkSaturationMatrix = <double>[
    1.39,
    -0.56,
    -0.11,
    0.00,
    0.30,
    -0.32,
    1.14,
    -0.11,
    0.00,
    0.30,
    -0.32,
    -0.56,
    1.59,
    0.00,
    0.30,
    0.00,
    0.00,
    0.00,
    1.00,
    0.00,
  ];

  /// Whether or not the area beneath this surface should be saturated with a
  /// [ColorFilter].
  ///
  /// The appearance of the [ColorFilter] is determined by the [Brightness]
  /// value obtained from the ambient [CupertinoTheme].
  ///
  /// The vibrance is always painted if asserts are disabled.
  ///
  /// Defaults to true.
  static bool debugIsVibrancePainted = true;

  ImageFilter? _buildFilter(Brightness? brightness) {
    bool isVibrancePainted = true;
    assert(() {
      isVibrancePainted = debugIsVibrancePainted;
      return true;
    }());
    if ((kIsWeb && !isSkiaWeb) || !isVibrancePainted) {
      if (blurSigma == 0) {
        return null;
      }
      return ImageFilter.blur(sigmaX: blurSigma, sigmaY: blurSigma);
    }

    final ColorFilter colorFilter = switch (brightness) {
      Brightness.dark => const ColorFilter.matrix(_darkSaturationMatrix),
      Brightness.light || null => const ColorFilter.matrix(_lightSaturationMatrix),
    };

    if (blurSigma == 0) {
      return colorFilter;
    }

    return ImageFilter.compose(
      inner: colorFilter,
      outer: ImageFilter.blur(sigmaX: blurSigma, sigmaY: blurSigma),
    );
  }

  @override
  Widget build(BuildContext context) {
    final ImageFilter? filter = _buildFilter(CupertinoTheme.maybeBrightnessOf(context));
    Widget contents = child;

    if (isSurfacePainted) {
      contents = ColoredBox(
        color: CupertinoDynamicColor.resolve(_kDialogColor, context),
        child: contents,
      );
    }

    if (filter != null) {
      return ClipRRect(
        borderRadius: _clipper,
        child: BackdropFilter(filter: filter, child: contents),
      );
    }

    return ClipRRect(borderRadius: _clipper, child: contents);
  }
}

typedef _HitTester = HitTestResult Function(Offset location);

// Recognizes taps with possible sliding during the tap.
//
// This recognizer only tracks one pointer at a time (called the primary
// pointer), and other pointers added while the primary pointer is alive are
// ignored and can not be used by other gestures either. After the primary
// pointer ends, the pointer added next becomes the new primary pointer (which
// starts a new gesture sequence).
//
// This recognizer only allows [kPrimaryMouseButton].
class _SlidingTapGestureRecognizer extends VerticalDragGestureRecognizer {
  _SlidingTapGestureRecognizer({super.debugOwner}) {
    dragStartBehavior = DragStartBehavior.down;
  }

  /// Called whenever the primary pointer moves regardless of whether drag has
  /// started.
  ///
  /// The parameter is the global position of the primary pointer.
  ///
  /// This is similar to `onUpdate`, but allows the caller to track the primary
  /// pointer's location before the drag starts, which is useful to enhance
  /// responsiveness.
  ValueSetter<Offset>? onResponsiveUpdate;

  /// Called whenever the primary pointer is lifted regardless of whether drag
  /// has started.
  ///
  /// The parameter is the global position of the primary pointer.
  ///
  /// This is similar to `onEnd`, but allows know the primary pointer's final
  /// location even if the drag never started, which is useful to enhance
  /// responsiveness.
  ValueSetter<Offset>? onResponsiveEnd;

  int? _primaryPointer;

  @override
  void addAllowedPointer(PointerDownEvent event) {
    _primaryPointer ??= event.pointer;
    super.addAllowedPointer(event);
  }

  @override
  void rejectGesture(int pointer) {
    if (pointer == _primaryPointer) {
      _primaryPointer = null;
    }
    super.rejectGesture(pointer);
  }

  @override
  void handleEvent(PointerEvent event) {
    if (event.pointer == _primaryPointer) {
      if (event is PointerMoveEvent) {
        onResponsiveUpdate?.call(event.position);
      }
      // Sliding tap needs to handle 'up' events differently compared to typical
      // drag gestures. If there's another gesture recognizer (like scrolling)
      // competing and the pointer hasn't moved beyond the tolerance limit
      // (slop), this gesture must still be accepted.
      //
      // Simply calling `accept()` here to handle this won't work because it
      // would break backward compatibility with legacy buttons (see
      // https://github.com/flutter/flutter/issues/150980 for more details).
      // Legacy buttons recognize taps using `GestureDetector.onTap`, which
      // neither accepts nor rejects for short taps. Instead, they wait for the
      // default resolution as the last contender in the gesture arena.
      //
      // Therefore, this gesture should also follow the same strategy of not
      // immediately accepting or rejecting. This allows tap gestures to take
      // precedence for being inner, while sliding taps can take precedence over
      // scroll gestures when the latter give up.
      if (event is PointerUpEvent) {
        stopTrackingPointer(_primaryPointer!);
        onResponsiveEnd?.call(event.position);
        _primaryPointer = null;
        // Do not call `super.handleEvent`, which gives up the pointer and thus
        // rejects the gesture.
        return;
      }
      if (event is PointerCancelEvent) {
        _primaryPointer = null;
      }
    }
    super.handleEvent(event);
  }

  @override
  String get debugDescription => 'tap slide';
}

// A region (typically a button) that can receive entering, exiting, and
// updating events of a "sliding tap" gesture.
//
// Some Cupertino widgets, such as action sheets or dialogs, allow the user to
// select buttons using "sliding taps", where the user can drag around after
// pressing on the screen, and whichever button the drag ends in is selected.
//
// This class is used to define the regions that sliding taps recognize. This
// class must be provided to a `MetaData` widget as `data`, and is typically
// implemented by a widget state class. When an eligible dragging gesture
// enters, leaves, or ends this `MetaData` widget, corresponding methods of this
// class will be called.
//
// Multiple `_SlideTarget`s might be nested.
// `_TargetSelectionGestureRecognizer` uses a simple algorithm that only
// compares if the inner-most slide target has changed (which suffices our use
// case). Semantically, this means that all outer targets will be treated as
// having the identical area as the inner-most one, i.e. when the pointer enters
// or leaves a slide target, the corresponding method will be called on all
// targets that nest it.
abstract class _SlideTarget {
  // A pointer has entered this region.
  //
  // This includes:
  //
  //  * The pointer has moved into this region from outside.
  //  * The point has contacted the screen in this region. In this case, this
  //    method is called as soon as the pointer down event occurs regardless of
  //    whether the gesture wins the arena immediately.
  //
  // The `fromPointerDown` should be true if this callback is triggered by a
  // PointerDownEvent, i.e. the second case from the list above.
  //
  // The return value of this method is used as the `innerEnabled` for the next
  // target, while `innerEnabled` of the innermost target is true.
  bool didEnter({required bool fromPointerDown, required bool innerEnabled});

  // A pointer has exited this region.
  //
  // This includes:
  //  * The pointer has moved out of this region.
  //  * The pointer is no longer in contact with the screen.
  //  * The pointer is canceled.
  //  * The gesture loses the arena.
  //  * The gesture ends. In this case, this method is called immediately
  //    before [didConfirm].
  void didLeave();

  // The drag gesture is completed in this region.
  //
  // This method is called immediately after a [didLeave].
  void didConfirm();
}

// Recognizes sliding taps and thereupon interacts with
// `_SlideTarget`s.
//
// TODO(dkwingsmt): It should recompute hit testing when the app is updated,
// or better, share code with `MouseTracker`.
// https://github.com/flutter/flutter/issues/155266
class _TargetSelectionGestureRecognizer extends GestureRecognizer {
  _TargetSelectionGestureRecognizer({super.debugOwner, required this.hitTest})
    : _slidingTap = _SlidingTapGestureRecognizer(debugOwner: debugOwner) {
    _slidingTap
      ..onDown = _onDown
      ..onResponsiveUpdate = _onUpdate
      ..onResponsiveEnd = _onEnd
      ..onCancel = _onCancel;
  }

  final _HitTester hitTest;

  final List<_SlideTarget> _currentTargets = <_SlideTarget>[];
  final _SlidingTapGestureRecognizer _slidingTap;

  @override
  void acceptGesture(int pointer) {
    _slidingTap.acceptGesture(pointer);
  }

  @override
  void rejectGesture(int pointer) {
    _slidingTap.rejectGesture(pointer);
  }

  @override
  void addPointer(PointerDownEvent event) {
    _slidingTap.addPointer(event);
  }

  @override
  void addPointerPanZoom(PointerPanZoomStartEvent event) {
    _slidingTap.addPointerPanZoom(event);
  }

  @override
  void dispose() {
    _slidingTap.dispose();
    super.dispose();
  }

  // Collect the `_SlideTarget`s that are currently hit by the
  // pointer, check whether the current target have changed, and invoke their
  // methods if necessary.
  //
  // The `fromPointerDown` should be true if this update is triggered by a
  // PointerDownEvent.
  void _updateDrag(Offset pointerPosition, {required bool fromPointerDown}) {
    final HitTestResult result = hitTest(pointerPosition);

    // A slide target might nest other targets, therefore multiple targets might
    // be found.
    final List<_SlideTarget> foundTargets = <_SlideTarget>[];
    for (final HitTestEntry entry in result.path) {
      if (entry.target case final RenderMetaData target) {
        if (target.metaData is _SlideTarget) {
          foundTargets.add(target.metaData as _SlideTarget);
        }
      }
    }

    // Compare whether the active target has changed by simply comparing the
    // first (inner-most) avatar of the nest, ignoring the cases where
    // _currentTargets intersect with foundTargets (see _SlideTarget's
    // document for more explanation).
    if (_currentTargets.firstOrNull != foundTargets.firstOrNull) {
      for (final _SlideTarget target in _currentTargets) {
        target.didLeave();
      }
      _currentTargets
        ..clear()
        ..addAll(foundTargets);
      bool enabled = true;
      for (final _SlideTarget target in _currentTargets) {
        enabled = target.didEnter(fromPointerDown: fromPointerDown, innerEnabled: enabled);
      }
    }
  }

  void _onDown(DragDownDetails details) {
    _updateDrag(details.globalPosition, fromPointerDown: true);
  }

  void _onUpdate(Offset globalPosition) {
    _updateDrag(globalPosition, fromPointerDown: false);
  }

  void _onEnd(Offset globalPosition) {
    _updateDrag(globalPosition, fromPointerDown: false);
    for (final _SlideTarget target in _currentTargets) {
      target.didConfirm();
    }
    _currentTargets.clear();
  }

  void _onCancel() {
    for (final _SlideTarget target in _currentTargets) {
      target.didLeave();
    }
    _currentTargets.clear();
  }

  @override
  String get debugDescription => 'target selection';
}

// The gesture detector used by action sheets.
//
// This gesture detector only recognizes one gesture,
// `_TargetSelectionGestureRecognizer`.
//
// This widget's child might contain another VerticalDragGestureRecognizer if
// the actions section or the content section scrolls. Conveniently, Flutter's
// gesture algorithm makes the inner gesture take priority.
class _ActionSheetGestureDetector extends StatelessWidget {
  const _ActionSheetGestureDetector({this.child});

  final Widget? child;

  HitTestResult _hitTest(BuildContext context, Offset globalPosition) {
    final int viewId = View.of(context).viewId;
    final HitTestResult result = HitTestResult();
    WidgetsBinding.instance.hitTestInView(result, globalPosition, viewId);
    return result;
  }

  @override
  Widget build(BuildContext context) {
    final Map<Type, GestureRecognizerFactory> gestures = <Type, GestureRecognizerFactory>{};
    gestures[_TargetSelectionGestureRecognizer] =
        GestureRecognizerFactoryWithHandlers<_TargetSelectionGestureRecognizer>(
          () => _TargetSelectionGestureRecognizer(
            debugOwner: this,
            hitTest: (Offset globalPosition) => _hitTest(context, globalPosition),
          ),
          (_TargetSelectionGestureRecognizer instance) {},
        );

    return RawGestureDetector(excludeFromSemantics: true, gestures: gestures, child: child);
  }
}

/// An iOS-style action sheet.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=U-ao8p4A82k}
///
/// An action sheet is a specific style of alert that presents the user
/// with a set of two or more choices related to the current context.
/// An action sheet can have a title, an additional message, and a list
/// of actions. The title is displayed above the message and the actions
/// are displayed below this content.
///
/// This action sheet styles its title and message to match standard iOS action
/// sheet title and message text style.
///
/// To display action buttons that look like standard iOS action sheet buttons,
/// provide [CupertinoActionSheetAction]s for the [actions] given to this action
/// sheet.
///
/// To include a iOS-style cancel button separate from the other buttons,
/// provide an [CupertinoActionSheetAction] for the [cancelButton] given to this
/// action sheet.
///
/// An action sheet is typically passed as the child widget to
/// [showCupertinoModalPopup], which displays the action sheet by sliding it up
/// from the bottom of the screen.
///
/// {@tool dartpad}
/// This sample shows how to use a [CupertinoActionSheet].
///	The [CupertinoActionSheet] shows a modal popup that slides in from the
/// bottom when [CupertinoButton] is pressed.
///
/// ** See code in examples/api/lib/cupertino/dialog/cupertino_action_sheet.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoActionSheetAction], which is an iOS-style action sheet button.
///  * <https://developer.apple.com/design/human-interface-guidelines/ios/views/action-sheets/>
class CupertinoActionSheet extends StatefulWidget {
  /// Creates an iOS-style action sheet.
  ///
  /// An action sheet must have a non-null value for at least one of the
  /// following arguments: [actions], [title], [message], or [cancelButton].
  ///
  /// Generally, action sheets are used to give the user a choice between
  /// two or more choices for the current context.
  const CupertinoActionSheet({
    super.key,
    this.title,
    this.message,
    this.actions,
    this.messageScrollController,
    this.actionScrollController,
    this.cancelButton,
  }) : assert(
         actions != null || title != null || message != null || cancelButton != null,
         'An action sheet must have a non-null value for at least one of the following arguments: '
         'actions, title, message, or cancelButton',
       );

  /// An optional title of the action sheet. When the [message] is non-null,
  /// the font of the [title] is bold.
  ///
  /// Typically a [Text] widget.
  final Widget? title;

  /// An optional descriptive message that provides more details about the
  /// reason for the alert.
  ///
  /// Typically a [Text] widget.
  final Widget? message;

  /// The set of actions that are displayed for the user to select.
  ///
  /// This must be a list of [CupertinoActionSheetAction] widgets.
  final List<Widget>? actions;

  /// A scroll controller that can be used to control the scrolling of the
  /// [message] in the action sheet.
  ///
  /// Defaults to null, which means the [CupertinoActionSheet] will create a
  /// scroll controller internally.
  final ScrollController? messageScrollController;

  /// A scroll controller that can be used to control the scrolling of the
  /// [actions] in the action sheet.
  ///
  /// Defaults to null, which means the [CupertinoActionSheet] will create an
  /// action scroll controller internally.
  final ScrollController? actionScrollController;

  /// The optional cancel button that is grouped separately from the other
  /// actions.
  ///
  /// This must be a [CupertinoActionSheetAction] widget.
  final Widget? cancelButton;

  @override
  State<CupertinoActionSheet> createState() => _CupertinoActionSheetState();
}

class _CupertinoActionSheetState extends State<CupertinoActionSheet> {
  int? _pressedIndex;
  static const int _kCancelButtonIndex = -1;

  ScrollController? _backupMessageScrollController;

  ScrollController? _backupActionScrollController;

  ScrollController get _effectiveMessageScrollController =>
      widget.messageScrollController ?? (_backupMessageScrollController ??= ScrollController());

  ScrollController get _effectiveActionScrollController =>
      widget.actionScrollController ?? (_backupActionScrollController ??= ScrollController());

  @override
  void dispose() {
    _backupMessageScrollController?.dispose();
    _backupActionScrollController?.dispose();
    super.dispose();
  }

  bool get hasContent => widget.title != null || widget.message != null;

  Widget? _buildContent(BuildContext context) {
    if (!hasContent) {
      return null;
    }
    final TextStyle textStyle = _kActionSheetContentStyle.copyWith(
      color: CupertinoDynamicColor.resolve(_kActionSheetContentTextColor, context),
    );
    return ColoredBox(
      color: CupertinoDynamicColor.resolve(_kActionSheetBackgroundColor, context),
      child: _CupertinoAlertContentSection(
        title: widget.title,
        message: widget.message,
        scrollController: _effectiveMessageScrollController,
        titlePadding: EdgeInsets.only(
          left: _kActionSheetContentHorizontalPadding,
          right: _kActionSheetContentHorizontalPadding,
          bottom: widget.message == null ? _kActionSheetContentVerticalPadding : 0.0,
          top: _kActionSheetContentVerticalPadding,
        ),
        messagePadding: EdgeInsets.only(
          left: _kActionSheetContentHorizontalPadding,
          right: _kActionSheetContentHorizontalPadding,
          bottom: _kActionSheetContentVerticalPadding,
          top: widget.title == null ? _kActionSheetContentVerticalPadding : 0.0,
        ),
        titleTextStyle:
            widget.message == null ? textStyle : textStyle.copyWith(fontWeight: FontWeight.w600),
        messageTextStyle:
            widget.title == null ? textStyle.copyWith(fontWeight: FontWeight.w600) : textStyle,
        additionalPaddingBetweenTitleAndMessage: const EdgeInsets.only(top: 4.0),
      ),
    );
  }

  void _onPressedUpdate(int actionIndex, bool state) {
    if (!state) {
      if (_pressedIndex == actionIndex) {
        setState(() {
          _pressedIndex = null;
        });
      }
    } else {
      setState(() {
        _pressedIndex = actionIndex;
      });
    }
  }

  Widget _buildCancelButton() {
    assert(widget.cancelButton != null);
    final double cancelPadding =
        (widget.actions != null || widget.message != null || widget.title != null)
            ? _kActionSheetCancelButtonPadding
            : 0.0;
    return Padding(
      padding: EdgeInsets.only(top: cancelPadding),
      child: _ActionSheetButtonBackground(
        isCancel: true,
        pressed: _pressedIndex == _kCancelButtonIndex,
        onPressStateChange: (bool state) {
          _onPressedUpdate(_kCancelButtonIndex, state);
        },
        child: widget.cancelButton!,
      ),
    );
  }

  // Given data point (x1, y1) and (x2, y2), derive the y corresponding to x
  // using linear interpolation between the two data points, and extrapolates
  // flatly beyond these points.
  //
  //              (x2, y2)
  //                _____________
  //               /
  //              /
  //    _________/
  //           (x1, y1)
  static double _lerp(double x, double x1, double y1, double x2, double y2) {
    if (x <= x1) {
      return y1;
    } else if (x >= x2) {
      return y2;
    } else {
      return lerpDouble(y1, y2, (x - x1) / (x2 - x1))!;
    }
  }

  // Derive the top padding, which is the distance between the top of a
  // full-height action sheet and the top of the safe area.
  //
  // The algorithm and its values are derived from measuring on the simulator.
  double _topPadding(BuildContext context) {
    if (MediaQuery.orientationOf(context) == Orientation.landscape) {
      return _kActionSheetEdgePadding;
    }

    // The top padding in portrait mode is in general close to the top view
    // padding, but not always equal:
    //
    //                            | view padding | action sheet padding | ratio
    //   No notch (eg. iPhone SE) |     20.0     |        20.0          | 1.0
    //   Notch (eg. iPhone 13)    |     47.0     |        47.0          | 1.0
    //   Capsule (eg. iPhone 15)  |     59.0     |        54.0          | 0.915
    //
    // Currently, we cannot determine why the result changes on "capsules."
    // Therefore, we'll hard code this rule, given the limited types of actual
    // devices. To provide an algorithm that accepts arbitrary view padding, this
    // function calculates the ratio as a continuous curve with linear
    // interpolation.

    // The x for lerp is the top view padding, while the y is ratio of
    // action sheet padding versus top view padding.
    const double viewPaddingData1 = 47.0;
    const double paddingRatioData1 = 1.0;
    const double viewPaddingData2 = 59.0;
    const double paddingRatioData2 = 54.0 / 59.0;

    final double currentViewPadding = MediaQuery.viewPaddingOf(context).top;

    final double currentPaddingRatio = _lerp(
      /* x= */ currentViewPadding,
      /* x1, y1= */ viewPaddingData1,
      paddingRatioData1,
      /* x2, y2= */ viewPaddingData2,
      paddingRatioData2,
    );
    final double padding = (currentPaddingRatio * currentViewPadding).roundToDouble();
    // In case there is no view padding, there should still be some space
    // between the action sheet and the edge.
    return math.max(padding, _kDialogEdgePadding);
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMediaQuery(context));

    /*
     *  ╭─────────────────╮  ↑                ↑
     *  │    The title    │ Content section   |
     *  │   The message   │  ↓                |
     *  ├─────────────────┤  ↑             Main sheet
     *  │    Action 1     │  |                |
     *  ├─────────────────┤ Actions section   |
     *  │    Action 2     │  |                |
     *  ╰─────────────────╯  ↓                ↓
     *  ╭─────────────────╮
     *  │     Cancel      │
     *  ╰─────────────────╯
     */

    final List<Widget> children = <Widget>[
      Flexible(
        child: ClipRRect(
          borderRadius: const BorderRadius.all(Radius.circular(12.0)),
          child: BackdropFilter(
            filter: ImageFilter.blur(
              sigmaX: CupertinoPopupSurface.defaultBlurSigma,
              sigmaY: CupertinoPopupSurface.defaultBlurSigma,
            ),
            child: _ActionSheetMainSheet(
              pressedIndex: _pressedIndex,
              onPressedUpdate: _onPressedUpdate,
              scrollController: _effectiveActionScrollController,
              contentSection: _buildContent(context),
              actions: widget.actions ?? List<Widget>.empty(),
              dividerColor: CupertinoDynamicColor.resolve(_kActionSheetButtonDividerColor, context),
            ),
          ),
        ),
      ),
      if (widget.cancelButton != null) _buildCancelButton(),
    ];
    final double actionSheetWidth = switch (MediaQuery.orientationOf(context)) {
      Orientation.portrait => MediaQuery.sizeOf(context).width,
      Orientation.landscape => MediaQuery.sizeOf(context).height,
    };

    return SafeArea(
      minimum: const EdgeInsets.only(bottom: _kActionSheetEdgePadding),
      child: ScrollConfiguration(
        // A CupertinoScrollbar is built-in below
        behavior: ScrollConfiguration.of(context).copyWith(scrollbars: false),
        child: Semantics(
          namesRoute: true,
          scopesRoute: true,
          explicitChildNodes: true,
          label: 'Alert',
          child: CupertinoUserInterfaceLevel(
            data: CupertinoUserInterfaceLevelData.elevated,
            child: Padding(
              padding: EdgeInsets.only(
                left: _kActionSheetEdgePadding,
                right: _kActionSheetEdgePadding,
                top: _topPadding(context),
                // The bottom padding is set on SafeArea.minimum, allowing it to
                // be consumed by bottom view padding.
              ),
              child: SizedBox(
                width: actionSheetWidth - _kActionSheetEdgePadding * 2,
                child: _ActionSheetGestureDetector(
                  child: Semantics(
                    explicitChildNodes: true,
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.end,
                      mainAxisSize: MainAxisSize.min,
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: children,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

/// The content of a typical action button in a [CupertinoActionSheet].
///
/// This widget draws the content of a button, i.e. the text, while the
/// background of the button is drawn by [CupertinoActionSheet].
///
/// See also:
///
///  * [CupertinoActionSheet], an alert that presents the user with a set of two or
///    more choices related to the current context.
class CupertinoActionSheetAction extends StatefulWidget {
  /// Creates an action for an iOS-style action sheet.
  const CupertinoActionSheetAction({
    super.key,
    required this.onPressed,
    this.isDefaultAction = false,
    this.isDestructiveAction = false,
    this.mouseCursor,
    required this.child,
  });

  /// The callback that is called when the button is selected.
  ///
  /// The button can be selected by either by tapping on this button or by
  /// pressing elsewhere and sliding onto this button before releasing.
  final VoidCallback onPressed;

  /// Whether this action is the default choice in the action sheet.
  ///
  /// Default buttons have bold text.
  final bool isDefaultAction;

  /// Whether this action might change or delete data.
  ///
  /// Destructive buttons have red text.
  final bool isDestructiveAction;

  /// The cursor that will be shown when hovering over the button.
  ///
  /// If null, defaults to [SystemMouseCursors.click] on web and
  /// [MouseCursor.defer] on other platforms.
  final MouseCursor? mouseCursor;

  /// The widget below this widget in the tree.
  ///
  /// Typically a [Text] widget.
  final Widget child;

  @override
  State<CupertinoActionSheetAction> createState() => _CupertinoActionSheetActionState();
}

class _CupertinoActionSheetActionState extends State<CupertinoActionSheetAction>
    implements _SlideTarget {
  // |_SlideTarget|
  @override
  bool didEnter({required bool fromPointerDown, required bool innerEnabled}) {
    return innerEnabled;
  }

  // |_SlideTarget|
  @override
  void didLeave() {}

  // |_SlideTarget|
  @override
  void didConfirm() {
    widget.onPressed();
  }

  // Calculates the font size for action sheet buttons.
  //
  // The `contextBodySize` is the body font size specified by context. The
  // return value is the button font size, including the effect of context font
  // scale factor. Divide by context font scale factor before using in a `Text`.
  static double _buttonFontSize(double contextBodySize) {
    // It is observed that the native action sheet buttons use font sizes that
    // deviate from standard HIG specifications in a non-linear way. The following
    // table shows the regular body font size vs the button font size:
    //
    //  Text scale  | xs |  s |  m |  l | xl | xxl | xxxl | ax1 | ax2 | ax3 | ax4 | ax5
    //  Body font   | 14 | 15 | 16 | 17 | 19 |  21 |  23  |  28 |  33 |  40 |  47 |  53
    //  Button font | 21 | 21 | 21 | 21 | 23 |  24 |  24  |  28 |  33 |  40 |  47 |  53

    // For very small or very large text, simple rules can be observed.
    // For mid-sized text, piecewise linear interpolation is used.
    return switch (contextBodySize) {
      <= 17 => 21.0,
      <= 19 => lerpDouble(21.0, 23.0, (contextBodySize - 17.0) / (19.0 - 17.0))!,
      <= 21 => lerpDouble(23.0, 24.0, (contextBodySize - 19.0) / (21.0 - 19.0))!,
      <= 24 => 24.0,
      _ => contextBodySize,
    };
  }

  @override
  Widget build(BuildContext context) {
    // The context scale factor is derived from the current body size and the
    // standard body size in "large".
    const double higLargeBodySize = 17.0;
    final double contextBodySize = MediaQuery.textScalerOf(context).scale(higLargeBodySize);
    final double contextScaleFactor = contextBodySize / higLargeBodySize;
    final double fontSize = _buttonFontSize(contextBodySize);

    TextStyle style = _kActionSheetActionStyle.copyWith(
      // `Text` will scale the provided font size inside, so its parameter is
      // unscaled first.
      fontSize: fontSize / contextScaleFactor,
      color:
          widget.isDestructiveAction
              ? CupertinoDynamicColor.resolve(CupertinoColors.systemRed, context)
              : CupertinoTheme.of(context).primaryColor,
    );

    if (widget.isDefaultAction) {
      style = style.copyWith(fontWeight: FontWeight.w600);
    }

    final double verticalPadding =
        _kActionSheetButtonVerticalPaddingBase +
        fontSize * _kActionSheetButtonVerticalPaddingFactor;

    return MouseRegion(
      cursor: widget.mouseCursor ?? (kIsWeb ? SystemMouseCursors.click : MouseCursor.defer),
      child: MetaData(
        metaData: this,
        behavior: HitTestBehavior.opaque,
        child: ConstrainedBox(
          constraints: const BoxConstraints(minHeight: _kActionSheetButtonMinHeight),
          child: Semantics(
            button: true,
            onTap: widget.onPressed,
            child: Padding(
              padding: EdgeInsets.fromLTRB(
                _kActionSheetButtonHorizontalPadding,
                verticalPadding,
                _kActionSheetButtonHorizontalPadding,
                verticalPadding,
              ),
              child: DefaultTextStyle(
                style: style,
                textAlign: TextAlign.center,
                child: Center(child: widget.child),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

// Renders the background of a button (both the pressed background and the idle
// background) and reports its state to the parent with `onPressStateChange`.
//
// Although this class doesn't keep any states, it's still a stateful widget
// because the state is used as a persistent object across rebuilds to provide
// to [MetaData.data].
class _ActionSheetButtonBackground extends StatefulWidget {
  const _ActionSheetButtonBackground({
    this.isCancel = false,
    required this.pressed,
    this.onPressStateChange,
    required this.child,
  });

  final bool isCancel;

  /// Whether the user is holding on this button.
  final bool pressed;

  /// Called when the user taps down or lifts up on the button.
  ///
  /// The boolean value is true if the user is tapping down on the button.
  final ValueSetter<bool>? onPressStateChange;

  /// The widget below this widget in the tree.
  ///
  /// Typically a [Text] widget.
  final Widget child;

  @override
  _ActionSheetButtonBackgroundState createState() => _ActionSheetButtonBackgroundState();
}

class _ActionSheetButtonBackgroundState extends State<_ActionSheetButtonBackground>
    implements _SlideTarget {
  void _emitVibration() {
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.android:
        HapticFeedback.selectionClick();
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        break;
    }
  }

  // |_SlideTarget|
  @override
  bool didEnter({required bool fromPointerDown, required bool innerEnabled}) {
    // Action sheet doesn't support disabled buttons, therefore `innerEnabled`
    // is always true.
    assert(innerEnabled);
    widget.onPressStateChange?.call(true);
    if (!fromPointerDown) {
      _emitVibration();
    }
    return innerEnabled;
  }

  // |_SlideTarget|
  @override
  void didLeave() {
    widget.onPressStateChange?.call(false);
  }

  // |_SlideTarget|
  @override
  void didConfirm() {
    widget.onPressStateChange?.call(false);
  }

  @override
  Widget build(BuildContext context) {
    late final Color backgroundColor;
    BorderRadius? borderRadius;
    if (!widget.isCancel) {
      backgroundColor = widget.pressed ? _kActionSheetPressedColor : _kActionSheetBackgroundColor;
    } else {
      backgroundColor = widget.pressed ? _kActionSheetCancelPressedColor : _kActionSheetCancelColor;
      borderRadius = const BorderRadius.all(Radius.circular(_kCornerRadius));
    }
    return MetaData(
      metaData: this,
      child: Container(
        decoration: BoxDecoration(
          color: CupertinoDynamicColor.resolve(backgroundColor, context),
          borderRadius: borderRadius,
        ),
        child: widget.child,
      ),
    );
  }
}

// The divider of an action sheet or an alert dialog.
//
// The divider can function as either a horizontal divider (in a column) or a
// vertical divider (in a row) without widget-layer configuration. Instead, this
// is determined during the layout phase based on the constraints. This approach
// is necessary to allow the alert dialog to provide a list of widgets to the
// layout widget, which doesn't know its layout mode until the layout phase.
//
// The constraints provided to this widget should match the column container's
// width or the row container's height, while being unlimited in the other
// dimension. This unlimited dimension will result in the divider's thickness.
//
// If the divider is not `hidden`, then it displays the `dividerColor`.
// Otherwise it displays the background color.
class _Divider extends StatelessWidget {
  const _Divider({required this.dividerColor, required this.hiddenColor, required this.hidden});

  final Color dividerColor;
  final Color hiddenColor;
  final bool hidden;

  @override
  Widget build(BuildContext context) {
    // The LimitedBox turns unconstrained dimension (typically the main axis of
    // a flex container) to the divider thickness.
    return LimitedBox(
      maxHeight: _kDividerThickness,
      maxWidth: _kDividerThickness,
      // The constrained box prevents the divider from collapsing to nothing.
      child: ConstrainedBox(
        constraints: const BoxConstraints(
          minHeight: _kDividerThickness,
          minWidth: _kDividerThickness,
        ),
        child: DecoratedBox(
          decoration: BoxDecoration(
            color: hidden ? CupertinoDynamicColor.resolve(hiddenColor, context) : dividerColor,
          ),
        ),
      ),
    );
  }
}

// Fills the overscroll area at the top or bottom of a scrollable widget with a
// solid color.
//
// This is necessary for action sheets and alert dialogs, because their actions
// section's background is rendered by the buttons, so that a button's
// background can be _replaced_ by a different color when the button is pressed.
class _OverscrollBackground extends StatefulWidget {
  const _OverscrollBackground({required this.color, required this.child});

  // The color for the overscroll part.
  //
  // This value must be a resolved color instead of, for example, a
  // CupertinoDynamicColor.
  final Color color;
  final Widget child;

  @override
  _OverscrollBackgroundState createState() => _OverscrollBackgroundState();
}

class _OverscrollBackgroundState extends State<_OverscrollBackground> {
  double _topOverscroll = 0;
  double _bottomOverscroll = 0;

  bool _onScrollUpdate(ScrollUpdateNotification notification) {
    final ScrollMetrics metrics = notification.metrics;
    setState(() {
      // The sizes of the overscroll should not be longer than the height of the
      // actions section.
      _topOverscroll = math.min(
        math.max(metrics.minScrollExtent - metrics.pixels, 0),
        metrics.viewportDimension,
      );
      _bottomOverscroll = math.min(
        math.max(metrics.pixels - metrics.maxScrollExtent, 0),
        metrics.viewportDimension,
      );
    });
    return false;
  }

  @override
  Widget build(BuildContext context) {
    final Widget overscroll = Column(
      mainAxisSize: MainAxisSize.min,
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: <Widget>[
        DecoratedBox(
          decoration: BoxDecoration(color: widget.color),
          child: SizedBox(height: _topOverscroll),
        ),
        DecoratedBox(
          decoration: BoxDecoration(color: widget.color),
          child: SizedBox(height: _bottomOverscroll),
        ),
      ],
    );
    return Stack(
      children: <Widget>[
        Positioned.fill(child: overscroll),
        NotificationListener<ScrollUpdateNotification>(
          onNotification: _onScrollUpdate,
          child: widget.child,
        ),
      ],
    );
  }
}

typedef _PressedUpdateHandler = void Function(int actionIndex, bool state);

// The list of actions in an action sheet.
//
// This excludes the divider between the action section and the content section.
class _ActionSheetActionSection extends StatelessWidget {
  const _ActionSheetActionSection({
    required this.actions,
    required this.pressedIndex,
    required this.dividerColor,
    required this.backgroundColor,
    required this.onPressedUpdate,
    required this.scrollController,
  });

  final List<Widget>? actions;
  final _PressedUpdateHandler onPressedUpdate;
  final int? pressedIndex;
  final Color dividerColor;
  final Color backgroundColor;
  final ScrollController scrollController;

  @override
  Widget build(BuildContext context) {
    if (actions == null || actions!.isEmpty) {
      return const LimitedBox(maxWidth: 0, child: SizedBox(width: double.infinity, height: 0));
    }
    final List<Widget> column = <Widget>[];
    for (int actionIndex = 0; actionIndex < actions!.length; actionIndex += 1) {
      if (actionIndex != 0) {
        column.add(
          _Divider(
            dividerColor: dividerColor,
            hiddenColor: _kActionSheetBackgroundColor,
            hidden: pressedIndex == actionIndex - 1 || pressedIndex == actionIndex,
          ),
        );
      }
      column.add(
        _ActionSheetButtonBackground(
          pressed: pressedIndex == actionIndex,
          onPressStateChange: (bool state) {
            onPressedUpdate(actionIndex, state);
          },
          child: actions![actionIndex],
        ),
      );
    }
    return CupertinoScrollbar(
      controller: scrollController,
      child: SingleChildScrollView(
        controller: scrollController,
        child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: column),
      ),
    );
  }
}

// The part of an action sheet without the cancel button.
class _ActionSheetMainSheet extends StatelessWidget {
  const _ActionSheetMainSheet({
    required this.pressedIndex,
    required this.onPressedUpdate,
    required this.scrollController,
    required this.actions,
    required this.contentSection,
    required this.dividerColor,
  });

  final int? pressedIndex;
  final _PressedUpdateHandler onPressedUpdate;
  final ScrollController scrollController;
  final List<Widget> actions;
  final Widget? contentSection;
  final Color dividerColor;

  Widget _scrolledActionsSection(BuildContext context) {
    final Color backgroundColor = CupertinoDynamicColor.resolve(
      _kActionSheetBackgroundColor,
      context,
    );
    return _OverscrollBackground(
      color: backgroundColor,
      child: _ActionSheetActionSection(
        actions: actions,
        scrollController: scrollController,
        dividerColor: dividerColor,
        backgroundColor: backgroundColor,
        pressedIndex: pressedIndex,
        onPressedUpdate: onPressedUpdate,
      ),
    );
  }

  Widget _dividerAndActionsSection(BuildContext context) {
    final Color backgroundColor = CupertinoDynamicColor.resolve(
      _kActionSheetBackgroundColor,
      context,
    );
    return Column(
      mainAxisSize: MainAxisSize.min,
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: <Widget>[
        _Divider(dividerColor: dividerColor, hiddenColor: backgroundColor, hidden: false),
        Flexible(child: _scrolledActionsSection(context)),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    if (actions.isEmpty) {
      return contentSection ?? _empty;
    }
    if (contentSection == null) {
      return _scrolledActionsSection(context);
    }
    return _PriorityColumn(
      top: contentSection!,
      bottom: _dividerAndActionsSection(context),
      bottomMinHeight: _kActionSheetActionsSectionMinHeight + _kDividerThickness,
    );
  }

  static const Widget _empty = LimitedBox(
    maxWidth: 0,
    child: SizedBox(width: double.infinity, height: 0),
  );
}

// The "content section" of a CupertinoAlertDialog.
//
// If title is missing, then only content is added. If content is
// missing, then only title is added. If both are missing, then it returns
// a SingleChildScrollView with a zero-sized Container.
class _CupertinoAlertContentSection extends StatelessWidget {
  const _CupertinoAlertContentSection({
    this.title,
    this.message,
    required this.scrollController,
    this.titlePadding,
    this.messagePadding,
    this.titleTextStyle,
    this.messageTextStyle,
    this.additionalPaddingBetweenTitleAndMessage,
  }) : assert(title == null || titlePadding != null && titleTextStyle != null),
       assert(message == null || messagePadding != null && messageTextStyle != null);

  // The (optional) title of the dialog is displayed in a large font at the top
  // of the dialog.
  //
  // Typically a Text widget.
  final Widget? title;

  // The (optional) message of the dialog is displayed in the center of the
  // dialog in a lighter font.
  //
  // Typically a Text widget.
  final Widget? message;

  // A scroll controller that can be used to control the scrolling of the
  // content in the dialog.
  final ScrollController scrollController;

  // Paddings used around title and message.
  // CupertinoAlertDialog and CupertinoActionSheet have different paddings.
  final EdgeInsets? titlePadding;
  final EdgeInsets? messagePadding;

  // Additional padding to be inserted between title and message.
  // Only used for CupertinoActionSheet.
  final EdgeInsets? additionalPaddingBetweenTitleAndMessage;

  // Text styles used for title and message.
  // CupertinoAlertDialog and CupertinoActionSheet have different text styles.
  final TextStyle? titleTextStyle;
  final TextStyle? messageTextStyle;

  @override
  Widget build(BuildContext context) {
    if (title == null && message == null) {
      return SingleChildScrollView(controller: scrollController, child: const SizedBox.shrink());
    }

    final List<Widget> titleContentGroup = <Widget>[
      if (title != null)
        Padding(
          padding: titlePadding!,
          child: DefaultTextStyle(
            style: titleTextStyle!,
            textAlign: TextAlign.center,
            child: title!,
          ),
        ),
      if (message != null)
        Padding(
          padding: messagePadding!,
          child: DefaultTextStyle(
            style: messageTextStyle!,
            textAlign: TextAlign.center,
            child: message!,
          ),
        ),
    ];

    // Add padding between the widgets if necessary.
    if (additionalPaddingBetweenTitleAndMessage != null && titleContentGroup.length > 1) {
      titleContentGroup.insert(1, Padding(padding: additionalPaddingBetweenTitleAndMessage!));
    }

    return CupertinoScrollbar(
      controller: scrollController,
      child: SingleChildScrollView(
        controller: scrollController,
        child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: titleContentGroup),
      ),
    );
  }
}

// The "actions section" of a [CupertinoAlertDialog].
//
// The `actions` must not be empty.
class _CupertinoAlertActionSection extends StatelessWidget {
  const _CupertinoAlertActionSection({
    required this.actions,
    required this.onPressedUpdate,
    required this.pressedIndex,
    required this.scrollController,
  }) : assert(actions.length != 0);

  // A list of action buttons.
  //
  // This list must not include the dividers between the buttons. If the list
  // is empty, then this widget returns an empty box.
  final List<Widget> actions;

  final _PressedUpdateHandler onPressedUpdate;
  final int? pressedIndex;

  // A scroll controller that can be used to control the scrolling of the
  // actions in the dialog.
  final ScrollController scrollController;

  @override
  Widget build(BuildContext context) {
    final Color dialogColor = CupertinoDynamicColor.resolve(_kDialogColor, context);
    final Color dialogPressedColor = CupertinoDynamicColor.resolve(_kDialogPressedColor, context);
    final Color dividerColor = CupertinoDynamicColor.resolve(CupertinoColors.separator, context);

    final List<Widget> column = <Widget>[];
    for (int actionIndex = 0; actionIndex < actions.length; actionIndex += 1) {
      if (actionIndex != 0) {
        column.add(
          _Divider(
            dividerColor: dividerColor,
            hiddenColor: dialogColor,
            hidden: pressedIndex == actionIndex - 1 || pressedIndex == actionIndex,
          ),
        );
      }
      column.add(
        _AlertDialogButtonBackground(
          idleColor: dialogColor,
          pressedColor: dialogPressedColor,
          pressed: pressedIndex == actionIndex,
          onPressStateChange: (bool state) {
            onPressedUpdate(actionIndex, state);
          },
          child: actions[actionIndex],
        ),
      );
    }

    return CupertinoScrollbar(
      controller: scrollController,
      child: SingleChildScrollView(
        controller: scrollController,
        child: _AlertDialogActionsLayout(dividerThickness: _kDividerThickness, children: column),
      ),
    );
  }
}

// Renders the background of a button (both the pressed background and the idle
// background) and reports its state to the parent with `onPressStateChange`.
class _AlertDialogButtonBackground extends StatefulWidget {
  const _AlertDialogButtonBackground({
    required this.idleColor,
    required this.pressedColor,
    required this.pressed,
    required this.onPressStateChange,
    required this.child,
  });

  /// Called whether the user is holding on this button.
  final bool pressed;

  /// Called when the user taps down or lifts up on the button.
  ///
  /// The boolean value is true if the user is tapping down on the button.
  final ValueSetter<bool>? onPressStateChange;

  final Color idleColor;
  final Color pressedColor;

  /// The widget below this widget in the tree.
  ///
  /// Typically a [Text] widget.
  final Widget child;

  @override
  _AlertDialogButtonBackgroundState createState() => _AlertDialogButtonBackgroundState();
}

class _AlertDialogButtonBackgroundState extends State<_AlertDialogButtonBackground>
    implements _SlideTarget {
  void _emitVibration() {
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.android:
        HapticFeedback.selectionClick();
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        break;
    }
  }

  // |_SlideTarget|
  @override
  bool didEnter({required bool fromPointerDown, required bool innerEnabled}) {
    widget.onPressStateChange?.call(innerEnabled);
    if (innerEnabled && !fromPointerDown) {
      _emitVibration();
    }
    return innerEnabled;
  }

  // |_SlideTarget|
  @override
  void didLeave() {
    widget.onPressStateChange?.call(false);
  }

  // |_SlideTarget|
  @override
  void didConfirm() {
    widget.onPressStateChange?.call(false);
  }

  @override
  Widget build(BuildContext context) {
    final Color backgroundColor = widget.pressed ? widget.pressedColor : widget.idleColor;
    return MetaData(
      metaData: this,
      child: MergeSemantics(
        child: Container(
          decoration: BoxDecoration(color: CupertinoDynamicColor.resolve(backgroundColor, context)),
          child: widget.child,
        ),
      ),
    );
  }
}

/// A button typically used in a [CupertinoAlertDialog].
///
/// See also:
///
///  * [CupertinoAlertDialog], a dialog that informs the user about situations
///    that require acknowledgment.
class CupertinoDialogAction extends StatefulWidget {
  /// Creates an action for an iOS-style dialog.
  const CupertinoDialogAction({
    super.key,
    this.onPressed,
    this.isDefaultAction = false,
    this.isDestructiveAction = false,
    this.textStyle,
    required this.child,
  });

  /// The callback that is called when the button is tapped or otherwise
  /// activated.
  ///
  /// If this is set to null, the button will be disabled.
  final VoidCallback? onPressed;

  /// Set to true if button is the default choice in the dialog.
  ///
  /// Default buttons have bold text. Similar to
  /// [UIAlertController.preferredAction](https://developer.apple.com/documentation/uikit/uialertcontroller/1620102-preferredaction),
  /// but more than one action can have this attribute set to true in the same
  /// [CupertinoAlertDialog].
  ///
  /// This parameters defaults to false.
  final bool isDefaultAction;

  /// Whether this action destroys an object.
  ///
  /// For example, an action that deletes an email is destructive.
  ///
  /// Defaults to false.
  final bool isDestructiveAction;

  /// [TextStyle] to apply to any text that appears in this button.
  ///
  /// Dialog actions have a built-in text resizing policy for long text. To
  /// ensure that this resizing policy always works as expected, [textStyle]
  /// must be used if a text size is desired other than that specified in
  /// [_kCupertinoDialogActionStyle].
  final TextStyle? textStyle;

  /// The widget below this widget in the tree.
  ///
  /// Typically a [Text] widget.
  final Widget child;

  @override
  State<CupertinoDialogAction> createState() => _CupertinoDialogActionState();
}

class _CupertinoDialogActionState extends State<CupertinoDialogAction> implements _SlideTarget {
  // The button is enabled when it has [onPressed].
  bool get enabled => widget.onPressed != null;

  // |_SlideTarget|
  @override
  bool didEnter({required bool fromPointerDown, required bool innerEnabled}) {
    return enabled;
  }

  // |_SlideTarget|
  @override
  void didLeave() {}

  // |_SlideTarget|
  @override
  void didConfirm() {
    widget.onPressed?.call();
  }

  // Dialog action content shrinks to fit, up to a certain point, and if it still
  // cannot fit at the minimum size, the text content is ellipsized.
  //
  // This policy only applies when the device is not in accessibility mode.
  Widget _buildContentWithRegularSizingPolicy({
    required BuildContext context,
    required TextStyle textStyle,
    required Widget content,
    required double padding,
  }) {
    final bool isInAccessibilityMode = _isInAccessibilityMode(context);
    final double dialogWidth =
        isInAccessibilityMode ? _kAccessibilityCupertinoDialogWidth : _kCupertinoDialogWidth;
    // The fontSizeRatio is the ratio of the current text size (including any
    // iOS scale factor) vs the minimum text size that we allow in action
    // buttons. This ratio information is used to automatically scale down action
    // button text to fit the available space.
    final double fontSizeRatio =
        MediaQuery.textScalerOf(context).scale(textStyle.fontSize!) / _kDialogMinButtonFontSize;

    return FittedBox(
      fit: BoxFit.scaleDown,
      child: ConstrainedBox(
        constraints: BoxConstraints(maxWidth: fontSizeRatio * (dialogWidth - (2 * padding))),
        child: Semantics(
          button: true,
          onTap: widget.onPressed,
          child: DefaultTextStyle(
            style: textStyle,
            textAlign: TextAlign.center,
            overflow: TextOverflow.ellipsis,
            maxLines: 1,
            child: content,
          ),
        ),
      ),
    );
  }

  // Dialog action content is permitted to be as large as it wants when in
  // accessibility mode. If text is used as the content, the text wraps instead
  // of ellipsizing.
  Widget _buildContentWithAccessibilitySizingPolicy({
    required TextStyle textStyle,
    required Widget content,
  }) {
    return DefaultTextStyle(style: textStyle, textAlign: TextAlign.center, child: content);
  }

  @override
  Widget build(BuildContext context) {
    TextStyle style = _kCupertinoDialogActionStyle
        .copyWith(
          color: CupertinoDynamicColor.resolve(
            widget.isDestructiveAction
                ? CupertinoColors.systemRed
                : CupertinoTheme.of(context).primaryColor,
            context,
          ),
        )
        .merge(widget.textStyle);

    if (widget.isDefaultAction) {
      style = style.copyWith(fontWeight: FontWeight.w600);
    }

    if (!enabled) {
      style = style.copyWith(color: style.color!.withOpacity(0.5));
    }
    final double fontSize = style.fontSize ?? kDefaultFontSize;
    final double fontSizeToScale = fontSize == 0.0 ? kDefaultFontSize : fontSize;
    final double effectiveTextScale =
        MediaQuery.textScalerOf(context).scale(fontSizeToScale) / fontSizeToScale;
    final double padding = 8.0 * effectiveTextScale;
    // Apply a sizing policy to the action button's content based on whether or
    // not the device is in accessibility mode.
    // TODO(mattcarroll): The following logic is not entirely correct. It is also
    // the case that if content text does not contain a space, it should also
    // wrap instead of ellipsizing. We are consciously not implementing that
    // now due to complexity.
    final Widget sizedContent =
        _isInAccessibilityMode(context)
            ? _buildContentWithAccessibilitySizingPolicy(textStyle: style, content: widget.child)
            : _buildContentWithRegularSizingPolicy(
              context: context,
              textStyle: style,
              content: widget.child,
              padding: padding,
            );

    return MouseRegion(
      cursor: widget.onPressed != null && kIsWeb ? SystemMouseCursors.click : MouseCursor.defer,
      child: MetaData(
        metaData: this,
        behavior: HitTestBehavior.opaque,
        child: ConstrainedBox(
          constraints: const BoxConstraints(minHeight: _kDialogMinButtonHeight),
          child: Padding(padding: EdgeInsets.all(padding), child: Center(child: sizedContent)),
        ),
      ),
    );
  }
}

// iOS style dialog action button layout.
//
// [_AlertDialogActionsLayout] does not provide any scrolling
// behavior for its buttons. It only handles the sizing and layout of buttons.
// Scrolling behavior can be composed on top of this widget, if desired.
//
// The layout operates in two modes:
//
// 1. Horizontal Mode: If there are exactly two buttons and they fit in a single
//    row, the buttons are rendered side by side with a vertical divider between
//    them.
// 2. Vertical Mode: In all other cases, the buttons are arranged in a column,
//    separated by horizontal dividers.
//
// The `children` parameter must be a non-empty list containing button widgets
// and divider widgets in an alternating sequence. Therefore, the list must have
// an odd length.
class _AlertDialogActionsLayout extends MultiChildRenderObjectWidget {
  const _AlertDialogActionsLayout({required double dividerThickness, required super.children})
    : _dividerThickness = dividerThickness;

  final double _dividerThickness;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderAlertDialogActionsLayout(dividerThickness: _dividerThickness);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderAlertDialogActionsLayout renderObject) {
    renderObject.dividerThickness = _dividerThickness;
  }
}

class _RenderAlertDialogActionsLayout extends RenderFlex {
  _RenderAlertDialogActionsLayout({List<RenderBox>? children, required double dividerThickness})
    : _dividerThickness = dividerThickness,
      super(
        direction: Axis.vertical,
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.stretch,
      ) {
    addAll(children);
  }

  // The thickness of the divider between buttons.
  double get dividerThickness => _dividerThickness;
  double _dividerThickness;
  set dividerThickness(double newValue) {
    if (newValue != _dividerThickness) {
      _dividerThickness = newValue;
      markNeedsLayout();
    }
  }

  double horizontalSlotWidthFor({required double overallWidth}) =>
      (overallWidth - dividerThickness) / 2;

  @override
  double computeMinIntrinsicHeight(double width) {
    if (!_useHorizontalLayout(width)) {
      return super.computeMinIntrinsicHeight(width);
    }

    final double slotWidth = horizontalSlotWidthFor(overallWidth: width);
    double height = 0;
    _forEachSlot((RenderBox slot) {
      height = math.max(height, slot.getMinIntrinsicHeight(slotWidth));
    });
    return height;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (!_useHorizontalLayout(width)) {
      return super.computeMaxIntrinsicHeight(width);
    }

    final double slotWidth = horizontalSlotWidthFor(overallWidth: width);
    double height = 0;
    _forEachSlot((RenderBox slot) {
      height = math.max(height, slot.getMaxIntrinsicHeight(slotWidth));
    });
    return height;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    if (!_debugHasValidConstraints(constraints)) {
      return Size.zero;
    }

    final double overallWidth = constraints.maxWidth;
    if (!_useHorizontalLayout(overallWidth)) {
      return super.computeDryLayout(constraints);
    }

    final double height = getMinIntrinsicHeight(overallWidth);
    return Size(overallWidth, height);
  }

  @override
  void performLayout() {
    if (firstChild == null) {
      size = constraints.smallest;
      return;
    }

    if (!_debugHasValidConstraints(constraints)) {
      size = constraints.smallest;
      return;
    }

    final double overallWidth = constraints.maxWidth;
    if (!_useHorizontalLayout(overallWidth)) {
      return super.performLayout();
    }

    final double slotWidth = horizontalSlotWidthFor(overallWidth: overallWidth);
    final double height = getMinIntrinsicHeight(overallWidth);
    size = Size(overallWidth, height);

    RenderBox slot = firstChild!;
    double x = 0;
    while (true) {
      slot.layout(BoxConstraints.tight(Size(slotWidth, height)), parentUsesSize: true);
      (slot.parentData! as FlexParentData).offset = Offset(x, 0);
      x += slot.size.width;

      final RenderBox? divider = childAfter(slot);
      if (divider == null) {
        break;
      }
      divider.layout(BoxConstraints.tight(Size(dividerThickness, height)));
      (divider.parentData! as FlexParentData).offset = Offset(x, 0);
      x += dividerThickness;

      slot = childAfter(divider)!;
    }
  }

  bool _debugHasValidConstraints(BoxConstraints constraints) {
    assert(() {
      ErrorSummary? errorSummary;
      if (constraints.maxWidth == double.infinity) {
        errorSummary = ErrorSummary('The incoming width constraints are unbounded.');
      }
      if (errorSummary != null) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          errorSummary,
          ErrorDescription('The incoming constraints are: $constraints'),
        ]);
      }
      return true;
    }());
    return true;
  }

  bool _useHorizontalLayout(double overallWidth) {
    // Horizontal layout only applies to cases of 3 children: 2 action buttons
    // and 1 divider.
    if (childCount != 3) {
      return false;
    }
    final double slotWidth = horizontalSlotWidthFor(overallWidth: overallWidth);
    RenderBox child = firstChild!;
    while (true) {
      // If both children fit into a half-row slot, use the horizontal layout.
      // Max intrinsic widths are used here, which, according to
      // [TextPainter.maxIntrinsicWidth], allows text to be displayed at their
      // full font size.
      if (child.getMaxIntrinsicWidth(double.infinity) > slotWidth) {
        return false;
      }
      final RenderBox? divider = childAfter(child);
      if (divider == null) {
        break;
      }
      child = childAfter(divider)!;
    }
    return true;
  }

  void _forEachSlot(ValueSetter<RenderBox> action) {
    assert(childCount.isOdd);
    RenderBox slot = firstChild!;
    while (true) {
      action(slot);
      final RenderBox? divider = childAfter(slot);
      if (divider == null) {
        break;
      }
      slot = childAfter(divider)!;
    }
  }
}

typedef _TwoChildrenHeights = ({double topChildHeight, double bottomChildHeight});

// A column layout with two widgets, where the top widget expands vertically as
// needed, and the bottom widget has a minimum height.
//
// Both child widgets stretch horizontally to the parent's maximum width
// constraint, with vertical space allocated in this priority:
//
//  1. The `bottom` widget receives its requested height, up to a
//     `bottomMaxHeight` limit and the container's constraint.
//  2. The `top` widget receives its requested height, up to the remaining space
//     in the container.
//  3. The `bottom` widget receives its requested height, up to any remaining
//     space in the container.
//
// This mirrors the behavior seen in iOS components like action sheets and
// alerts.
//
// Implementing this layout with simple compositing widgets is challenging
// because:
//
//  * The bottom widget should take more than `bottomMinHeight` if the top
//    widget is short.
//  * The bottom widget should take less than `bottomMinHeight` if it is
//    naturally shorter.
class _PriorityColumn extends MultiChildRenderObjectWidget {
  _PriorityColumn({required Widget top, required Widget bottom, required this.bottomMinHeight})
    : super(children: <Widget>[top, bottom]);

  final double bottomMinHeight;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderPriorityColumn(bottomMinHeight: bottomMinHeight);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderPriorityColumn renderObject) {
    renderObject.bottomMinHeight = bottomMinHeight;
  }
}

class _RenderPriorityColumn extends RenderFlex {
  _RenderPriorityColumn({List<RenderBox>? children, required double bottomMinHeight})
    : _bottomMinHeight = bottomMinHeight,
      super(
        direction: Axis.vertical,
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.stretch,
      ) {
    addAll(children);
  }

  double get bottomMinHeight => _bottomMinHeight;
  double _bottomMinHeight;
  set bottomMinHeight(double newValue) {
    if (newValue != _bottomMinHeight) {
      _bottomMinHeight = newValue;
      markNeedsLayout();
    }
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    assert(childCount == 2);
    return firstChild!.getMinIntrinsicHeight(width) + lastChild!.getMinIntrinsicHeight(width);
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    assert(childCount == 2);
    return firstChild!.getMaxIntrinsicHeight(width) + lastChild!.getMaxIntrinsicHeight(width);
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    final double width = constraints.maxWidth;
    final double maxHeight = constraints.maxHeight;
    final (:double topChildHeight, :double bottomChildHeight) = _childrenHeights(width, maxHeight);
    return Size(width, topChildHeight + bottomChildHeight);
  }

  @override
  void performLayout() {
    final double width = constraints.maxWidth;
    final double maxHeight = constraints.maxHeight;
    final (:double topChildHeight, :double bottomChildHeight) = _childrenHeights(width, maxHeight);
    size = Size(width, topChildHeight + bottomChildHeight);

    firstChild!.layout(BoxConstraints.tight(Size(width, topChildHeight)), parentUsesSize: true);
    (firstChild!.parentData! as FlexParentData).offset = Offset.zero;

    lastChild!.layout(BoxConstraints.tight(Size(width, bottomChildHeight)), parentUsesSize: true);
    (lastChild!.parentData! as FlexParentData).offset = Offset(0, topChildHeight);
  }

  _TwoChildrenHeights _childrenHeights(double width, double maxHeight) {
    assert(childCount == 2);
    final double topIntrinsic = firstChild!.getMinIntrinsicHeight(width);
    final double bottomIntrinsic = lastChild!.getMinIntrinsicHeight(width);
    // Try to layout both children as their intrinsic height.
    if (topIntrinsic + bottomIntrinsic <= maxHeight) {
      return (topChildHeight: topIntrinsic, bottomChildHeight: bottomIntrinsic);
    }
    // _bottomMinHeight is only effective when bottom actually needs that much.
    final double effectiveBottomMinHeight = math.min(_bottomMinHeight, bottomIntrinsic);
    // Try to layout top as intrinsics, as long as the bottom has at least
    // effectiveBottomMinHeight.
    if (maxHeight - topIntrinsic >= effectiveBottomMinHeight) {
      return (topChildHeight: topIntrinsic, bottomChildHeight: maxHeight - topIntrinsic);
    }
    // Try to layout bottom as effectiveBottomMinHeight, as long as top has at
    // least 0.
    if (maxHeight >= effectiveBottomMinHeight) {
      return (
        topChildHeight: maxHeight - effectiveBottomMinHeight,
        bottomChildHeight: effectiveBottomMinHeight,
      );
    }
    return (topChildHeight: 0, bottomChildHeight: maxHeight);
  }
}
