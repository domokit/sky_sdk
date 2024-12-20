// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'color_scheme.dart';
import 'colors.dart';
import 'debug.dart';
import 'icons.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'text_button.dart';
import 'text_theme.dart';
import 'theme.dart';

// TODO(dragostis): Missing functionality:
//   * mobile horizontal mode with adding/removing steps
//   * alternative labeling
//   * stepper feedback in the case of high-latency interactions

/// The state of a [Step] which is used to control the style of the circle and
/// text.
///
/// See also:
///
///  * [Step]
enum StepState {
  /// A step that displays its index in its circle.
  indexed,

  /// A step that displays a pencil icon in its circle.
  editing,

  /// A step that displays a tick icon in its circle.
  complete,

  /// A step that is disabled and does not to react to taps.
  disabled,

  /// A step that is currently having an error. e.g. the user has submitted wrong
  /// input.
  error,
}

/// Defines the [Stepper]'s main axis.
enum StepperType {
  /// A vertical layout of the steps with their content in-between the titles.
  vertical,

  /// A horizontal layout of the steps with their content below the titles.
  horizontal,
}

/// Container for all the information necessary to build a Stepper widget's
/// forward and backward controls for any given step.
///
/// Used by [Stepper.controlsBuilder].
@immutable
class ControlsDetails {
  /// Creates a set of details describing the Stepper.
  const ControlsDetails({
    required this.currentStep,
    required this.stepIndex,
    this.onStepCancel,
    this.onStepContinue,
  });

  /// Index that is active for the surrounding [Stepper] widget. This may be
  /// different from [stepIndex] if the user has just changed steps and we are
  /// currently animating toward that step.
  final int currentStep;

  /// Index of the step for which these controls are being built. This is
  /// not necessarily the active index, if the user has just changed steps and
  /// this step is animating away. To determine whether a given builder is building
  /// the active step or the step being navigated away from, see [isActive].
  final int stepIndex;

  /// The callback called when the 'continue' button is tapped.
  ///
  /// If null, the 'continue' button will be disabled.
  final VoidCallback? onStepContinue;

  /// The callback called when the 'cancel' button is tapped.
  ///
  /// If null, the 'cancel' button will be disabled.
  final VoidCallback? onStepCancel;

  /// True if the indicated step is also the current active step. If the user has
  /// just activated the transition to a new step, some [Stepper.type] values will
  /// lead to both steps being rendered for the duration of the animation shifting
  /// between steps.
  bool get isActive => currentStep == stepIndex;
}

/// A builder that creates a widget given the two callbacks `onStepContinue` and
/// `onStepCancel`.
///
/// Used by [Stepper.controlsBuilder].
///
/// See also:
///
///  * [WidgetBuilder], which is similar but only takes a [BuildContext].
typedef ControlsWidgetBuilder = Widget Function(BuildContext context, ControlsDetails details);

/// A builder that creates the icon widget for the [Step] at [stepIndex], given
/// [stepState].
typedef StepIconBuilder = Widget? Function(int stepIndex, StepState stepState);

const TextStyle _kStepStyle = TextStyle(fontSize: 12.0, color: Colors.white);
const Color _kErrorLight = Colors.red;
final Color _kErrorDark = Colors.red.shade400;
const Color _kCircleActiveLight = Colors.white;
const Color _kCircleActiveDark = Colors.black87;
const Color _kDisabledLight = Colors.black38;
const Color _kDisabledDark = Colors.white38;
const double _kStepSize = 24.0;
const double _kTriangleSqrt = 0.866025; // sqrt(3.0) / 2.0
const double _kTriangleHeight = _kStepSize * _kTriangleSqrt;
const double _kMaxStepSize = 80.0;

/// A material step used in [Stepper]. The step can have a title and subtitle,
/// an icon within its circle, some content and a state that governs its
/// styling.
///
/// See also:
///
///  * [Stepper]
///  * <https://material.io/archive/guidelines/components/steppers.html>
@immutable
class Step {
  /// Creates a step for a [Stepper].
  const Step({
    required this.title,
    this.subtitle,
    required this.content,
    this.state = StepState.indexed,
    this.isActive = false,
    this.label,
    this.stepStyle,
  });

  /// The title of the step that typically describes it.
  final Widget title;

  /// The subtitle of the step that appears below the title and has a smaller
  /// font size. It typically gives more details that complement the title.
  ///
  /// If null, the subtitle is not shown.
  final Widget? subtitle;

  /// The content of the step that appears below the [title] and [subtitle].
  ///
  /// Below the content, every step has a 'continue' and 'cancel' button.
  final Widget content;

  /// The state of the step which determines the styling of its components
  /// and whether steps are interactive.
  final StepState state;

  /// Whether or not the step is active. The flag only influences styling.
  final bool isActive;

  /// Only [StepperType.horizontal], Optional widget that appears under the [title].
  /// By default, uses the `bodyLarge` theme.
  final Widget? label;

  /// Optional overrides for the step's default visual configuration.
  final StepStyle? stepStyle;
}

/// A material stepper widget that displays progress through a sequence of
/// steps. Steppers are particularly useful in the case of forms where one step
/// requires the completion of another one, or where multiple steps need to be
/// completed in order to submit the whole form.
///
/// The widget is a flexible wrapper. A parent class should pass [currentStep]
/// to this widget based on some logic triggered by the three callbacks that it
/// provides.
///
/// {@tool dartpad}
/// An example the shows how to use the [Stepper], and the [Stepper] UI
/// appearance.
///
/// ** See code in examples/api/lib/material/stepper/stepper.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Step]
///  * <https://material.io/archive/guidelines/components/steppers.html>
class Stepper extends StatefulWidget {
  /// Creates a stepper from a list of steps.
  ///
  /// This widget is not meant to be rebuilt with a different list of steps
  /// unless a key is provided in order to distinguish the old stepper from the
  /// new one.
  const Stepper({
    super.key,
    required this.steps,
    this.controller,
    this.physics,
    this.type = StepperType.vertical,
    this.currentStep = 0,
    this.onStepTapped,
    this.onStepContinue,
    this.onStepCancel,
    this.controlsBuilder,
    this.elevation,
    this.margin,
    this.connectorColor,
    this.connectorThickness,
    this.stepIconBuilder,
    this.stepIconHeight,
    this.stepIconWidth,
    this.stepIconMargin,
    this.clipBehavior = Clip.none,
  }) : assert(0 <= currentStep && currentStep < steps.length),
       assert(
         stepIconHeight == null ||
             (stepIconHeight >= _kStepSize && stepIconHeight <= _kMaxStepSize),
         'stepIconHeight must be greater than $_kStepSize and less or equal to $_kMaxStepSize',
       ),
       assert(
         stepIconWidth == null || (stepIconWidth >= _kStepSize && stepIconWidth <= _kMaxStepSize),
         'stepIconWidth must be greater than $_kStepSize and less or equal to $_kMaxStepSize',
       ),
       assert(
         stepIconHeight == null || stepIconWidth == null || stepIconHeight == stepIconWidth,
         'If either stepIconHeight or stepIconWidth is specified, both must be specified and '
         'the values must be equal.',
       );

  /// The steps of the stepper whose titles, subtitles, icons always get shown.
  ///
  /// The length of [steps] must not change.
  final List<Step> steps;

  /// How the stepper's scroll view should respond to user input.
  ///
  /// For example, determines how the scroll view continues to
  /// animate after the user stops dragging the scroll view.
  ///
  /// If the stepper is contained within another scrollable it
  /// can be helpful to set this property to [ClampingScrollPhysics].
  final ScrollPhysics? physics;

  /// An object that can be used to control the position to which this scroll
  /// view is scrolled.
  ///
  /// To control the initial scroll offset of the scroll view, provide a
  /// [controller] with its [ScrollController.initialScrollOffset] property set.
  final ScrollController? controller;

  /// The type of stepper that determines the layout. In the case of
  /// [StepperType.horizontal], the content of the current step is displayed
  /// underneath as opposed to the [StepperType.vertical] case where it is
  /// displayed in-between.
  final StepperType type;

  /// The index into [steps] of the current step whose content is displayed.
  final int currentStep;

  /// The callback called when a step is tapped, with its index passed as
  /// an argument.
  final ValueChanged<int>? onStepTapped;

  /// The callback called when the 'continue' button is tapped.
  ///
  /// If null, the 'continue' button will be disabled.
  final VoidCallback? onStepContinue;

  /// The callback called when the 'cancel' button is tapped.
  ///
  /// If null, the 'cancel' button will be disabled.
  final VoidCallback? onStepCancel;

  /// The callback for creating custom controls.
  ///
  /// If null, the default controls from the current theme will be used.
  ///
  /// This callback which takes in a context and a [ControlsDetails] object, which
  /// contains step information and two functions: [onStepContinue] and [onStepCancel].
  /// These can be used to control the stepper. For example, reading the
  /// [ControlsDetails.currentStep] value within the callback can change the text
  /// of the continue or cancel button depending on which step users are at.
  ///
  /// {@tool dartpad}
  /// Creates a stepper control with custom buttons.
  ///
  /// ```dart
  /// Widget build(BuildContext context) {
  ///   return Stepper(
  ///     controlsBuilder:
  ///       (BuildContext context, ControlsDetails details) {
  ///          return Row(
  ///            children: <Widget>[
  ///              TextButton(
  ///                onPressed: details.onStepContinue,
  ///                child: Text('Continue to Step ${details.stepIndex + 1}'),
  ///              ),
  ///              TextButton(
  ///                onPressed: details.onStepCancel,
  ///                child: Text('Back to Step ${details.stepIndex - 1}'),
  ///              ),
  ///            ],
  ///          );
  ///       },
  ///     steps: const <Step>[
  ///       Step(
  ///         title: Text('A'),
  ///         content: SizedBox(
  ///           width: 100.0,
  ///           height: 100.0,
  ///         ),
  ///       ),
  ///       Step(
  ///         title: Text('B'),
  ///         content: SizedBox(
  ///           width: 100.0,
  ///           height: 100.0,
  ///         ),
  ///       ),
  ///     ],
  ///   );
  /// }
  /// ```
  /// ** See code in examples/api/lib/material/stepper/stepper.controls_builder.0.dart **
  /// {@end-tool}
  final ControlsWidgetBuilder? controlsBuilder;

  /// The elevation of this stepper's [Material] when [type] is [StepperType.horizontal].
  final double? elevation;

  /// Custom margin on vertical stepper.
  final EdgeInsetsGeometry? margin;

  /// Customize connected lines colors.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.selected].
  ///  * [WidgetState.disabled].
  ///
  /// If not set then the widget will use default colors, primary for selected state
  /// and grey.shade400 for disabled state.
  final MaterialStateProperty<Color>? connectorColor;

  /// The thickness of the connecting lines.
  final double? connectorThickness;

  /// Callback for creating custom icons for the [steps].
  ///
  /// When overriding icon for [StepState.error], please return
  /// a widget whose width and height are 14 pixels or less to avoid overflow.
  ///
  /// If null, the default icons will be used for respective [StepState].
  final StepIconBuilder? stepIconBuilder;

  /// Overrides the default step icon size height.
  final double? stepIconHeight;

  /// Overrides the default step icon size width.
  final double? stepIconWidth;

  /// Overrides the default step icon margin.
  final EdgeInsets? stepIconMargin;

  /// The [Step.content] will be clipped to this Clip type.
  ///
  /// Defaults to [Clip.none].
  ///
  /// See also:
  ///
  ///  * [Clip], which explains how to use this property.
  final Clip clipBehavior;

  @override
  State<Stepper> createState() => _StepperState();
}

class _StepperState extends State<Stepper> with TickerProviderStateMixin {
  late List<GlobalKey> _keys;
  final Map<int, StepState> _oldStates = <int, StepState>{};

  @override
  void initState() {
    super.initState();
    _keys = List<GlobalKey>.generate(widget.steps.length, (int i) => GlobalKey());

    for (int i = 0; i < widget.steps.length; i += 1) {
      _oldStates[i] = widget.steps[i].state;
    }
  }

  @override
  void didUpdateWidget(Stepper oldWidget) {
    super.didUpdateWidget(oldWidget);
    assert(widget.steps.length == oldWidget.steps.length);

    for (int i = 0; i < oldWidget.steps.length; i += 1) {
      _oldStates[i] = oldWidget.steps[i].state;
    }
  }

  EdgeInsetsGeometry? get _stepIconMargin => widget.stepIconMargin;

  double? get _stepIconHeight => widget.stepIconHeight;

  double? get _stepIconWidth => widget.stepIconWidth;

  double get _heightFactor {
    return (_isLabel() && _stepIconHeight != null) ? 2.5 : 2.0;
  }

  bool _isFirst(int index) {
    return index == 0;
  }

  bool _isLast(int index) {
    return widget.steps.length - 1 == index;
  }

  bool _isCurrent(int index) {
    return widget.currentStep == index;
  }

  bool _isDark() {
    return Theme.of(context).brightness == Brightness.dark;
  }

  bool _isLabel() {
    for (final Step step in widget.steps) {
      if (step.label != null) {
        return true;
      }
    }
    return false;
  }

  StepStyle? _stepStyle(int index) {
    return widget.steps[index].stepStyle;
  }

  Color _connectorColor(bool isActive) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final Set<MaterialState> states = <MaterialState>{
      if (isActive) MaterialState.selected else MaterialState.disabled,
    };
    final Color? resolvedConnectorColor = widget.connectorColor?.resolve(states);

    return resolvedConnectorColor ?? (isActive ? colorScheme.primary : Colors.grey.shade400);
  }

  Widget _buildLine(bool visible, bool isActive) {
    return ColoredBox(
      color: _connectorColor(isActive),
      child: SizedBox(width: visible ? widget.connectorThickness ?? 1.0 : 0.0, height: 16.0),
    );
  }

  Widget _buildCircleChild(int index, bool oldState) {
    final StepState state = oldState ? _oldStates[index]! : widget.steps[index].state;
    if (widget.stepIconBuilder?.call(index, state) case final Widget icon) {
      return icon;
    }
    TextStyle? textStyle = _stepStyle(index)?.indexStyle;
    final bool isDarkActive = _isDark() && widget.steps[index].isActive;
    final Color iconColor = isDarkActive ? _kCircleActiveDark : _kCircleActiveLight;
    textStyle ??= isDarkActive ? _kStepStyle.copyWith(color: Colors.black87) : _kStepStyle;

    return switch (state) {
      StepState.indexed || StepState.disabled => Text('${index + 1}', style: textStyle),
      StepState.editing => Icon(Icons.edit, color: iconColor, size: 18.0),
      StepState.complete => Icon(Icons.check, color: iconColor, size: 18.0),
      StepState.error => const Center(child: Text('!', style: _kStepStyle)),
    };
  }

  Color _circleColor(int index) {
    final bool isActive = widget.steps[index].isActive;
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final Set<MaterialState> states = <MaterialState>{
      if (isActive) MaterialState.selected else MaterialState.disabled,
    };
    final Color? resolvedConnectorColor = widget.connectorColor?.resolve(states);
    if (resolvedConnectorColor != null) {
      return resolvedConnectorColor;
    }
    if (!_isDark()) {
      return isActive ? colorScheme.primary : colorScheme.onSurface.withOpacity(0.38);
    } else {
      return isActive ? colorScheme.secondary : colorScheme.background;
    }
  }

  Widget _buildCircle(int index, bool oldState) {
    return Padding(
      padding: _stepIconMargin ?? const EdgeInsets.symmetric(vertical: 8.0),
      child: SizedBox(
        width: _stepIconWidth ?? _kStepSize,
        height: _stepIconHeight ?? _kStepSize,
        child: AnimatedContainer(
          curve: Curves.fastOutSlowIn,
          duration: kThemeAnimationDuration,
          decoration: BoxDecoration(
            color: _stepStyle(index)?.color ?? _circleColor(index),
            shape: BoxShape.circle,
            border: _stepStyle(index)?.border,
            boxShadow:
                _stepStyle(index)?.boxShadow != null
                    ? <BoxShadow>[_stepStyle(index)!.boxShadow!]
                    : null,
            gradient: _stepStyle(index)?.gradient,
          ),
          child: Center(
            child: _buildCircleChild(
              index,
              oldState && widget.steps[index].state == StepState.error,
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildTriangle(int index, bool oldState) {
    Color? color = _stepStyle(index)?.errorColor;
    color ??= _isDark() ? _kErrorDark : _kErrorLight;

    return Padding(
      padding: _stepIconMargin ?? const EdgeInsets.symmetric(vertical: 8.0),
      child: SizedBox(
        width: _stepIconWidth ?? _kStepSize,
        height: _stepIconHeight ?? _kStepSize,
        child: Center(
          child: SizedBox(
            width: _stepIconWidth ?? _kStepSize,
            height: _stepIconHeight != null ? _stepIconHeight! * _kTriangleSqrt : _kTriangleHeight,
            child: CustomPaint(
              painter: _TrianglePainter(color: color),
              child: Align(
                alignment: const Alignment(0.0, 0.8), // 0.8 looks better than the geometrical 0.33.
                child: _buildCircleChild(
                  index,
                  oldState && widget.steps[index].state != StepState.error,
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildIcon(int index) {
    if (widget.steps[index].state != _oldStates[index]) {
      return AnimatedCrossFade(
        firstChild: _buildCircle(index, true),
        secondChild: _buildTriangle(index, true),
        firstCurve: const Interval(0.0, 0.6, curve: Curves.fastOutSlowIn),
        secondCurve: const Interval(0.4, 1.0, curve: Curves.fastOutSlowIn),
        sizeCurve: Curves.fastOutSlowIn,
        crossFadeState:
            widget.steps[index].state == StepState.error
                ? CrossFadeState.showSecond
                : CrossFadeState.showFirst,
        duration: kThemeAnimationDuration,
      );
    } else {
      if (widget.steps[index].state != StepState.error) {
        return _buildCircle(index, false);
      } else {
        return _buildTriangle(index, false);
      }
    }
  }

  Widget _buildVerticalControls(int stepIndex) {
    if (widget.controlsBuilder != null) {
      return widget.controlsBuilder!(
        context,
        ControlsDetails(
          currentStep: widget.currentStep,
          onStepContinue: widget.onStepContinue,
          onStepCancel: widget.onStepCancel,
          stepIndex: stepIndex,
        ),
      );
    }

    final Color cancelColor = switch (Theme.of(context).brightness) {
      Brightness.light => Colors.black54,
      Brightness.dark => Colors.white70,
    };

    final ThemeData themeData = Theme.of(context);
    final ColorScheme colorScheme = themeData.colorScheme;
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);

    const OutlinedBorder buttonShape = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(2)),
    );
    const EdgeInsets buttonPadding = EdgeInsets.symmetric(horizontal: 16.0);

    return Padding(
      padding: const EdgeInsets.only(top: 16.0),
      child: SizedBox(
        height: 48.0,
        child: Row(
          // The Material spec no longer includes a Stepper widget. The continue
          // and cancel button styles have been configured to match the original
          // version of this widget.
          children: <Widget>[
            TextButton(
              onPressed: widget.onStepContinue,
              style: ButtonStyle(
                foregroundColor: MaterialStateProperty.resolveWith<Color?>((
                  Set<MaterialState> states,
                ) {
                  return states.contains(MaterialState.disabled)
                      ? null
                      : (_isDark() ? colorScheme.onSurface : colorScheme.onPrimary);
                }),
                backgroundColor: MaterialStateProperty.resolveWith<Color?>((
                  Set<MaterialState> states,
                ) {
                  return _isDark() || states.contains(MaterialState.disabled)
                      ? null
                      : colorScheme.primary;
                }),
                padding: const MaterialStatePropertyAll<EdgeInsetsGeometry>(buttonPadding),
                shape: const MaterialStatePropertyAll<OutlinedBorder>(buttonShape),
              ),
              child: Text(
                themeData.useMaterial3
                    ? localizations.continueButtonLabel
                    : localizations.continueButtonLabel.toUpperCase(),
              ),
            ),
            Padding(
              padding: const EdgeInsetsDirectional.only(start: 8.0),
              child: TextButton(
                onPressed: widget.onStepCancel,
                style: TextButton.styleFrom(
                  foregroundColor: cancelColor,
                  padding: buttonPadding,
                  shape: buttonShape,
                ),
                child: Text(
                  themeData.useMaterial3
                      ? localizations.cancelButtonLabel
                      : localizations.cancelButtonLabel.toUpperCase(),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  TextStyle _titleStyle(int index) {
    final ThemeData themeData = Theme.of(context);
    final TextTheme textTheme = themeData.textTheme;

    switch (widget.steps[index].state) {
      case StepState.indexed:
      case StepState.editing:
      case StepState.complete:
        return textTheme.bodyLarge!;
      case StepState.disabled:
        return textTheme.bodyLarge!.copyWith(color: _isDark() ? _kDisabledDark : _kDisabledLight);
      case StepState.error:
        return textTheme.bodyLarge!.copyWith(color: _isDark() ? _kErrorDark : _kErrorLight);
    }
  }

  TextStyle _subtitleStyle(int index) {
    final ThemeData themeData = Theme.of(context);
    final TextTheme textTheme = themeData.textTheme;

    switch (widget.steps[index].state) {
      case StepState.indexed:
      case StepState.editing:
      case StepState.complete:
        return textTheme.bodySmall!;
      case StepState.disabled:
        return textTheme.bodySmall!.copyWith(color: _isDark() ? _kDisabledDark : _kDisabledLight);
      case StepState.error:
        return textTheme.bodySmall!.copyWith(color: _isDark() ? _kErrorDark : _kErrorLight);
    }
  }

  TextStyle _labelStyle(int index) {
    final ThemeData themeData = Theme.of(context);
    final TextTheme textTheme = themeData.textTheme;

    switch (widget.steps[index].state) {
      case StepState.indexed:
      case StepState.editing:
      case StepState.complete:
        return textTheme.bodyLarge!;
      case StepState.disabled:
        return textTheme.bodyLarge!.copyWith(color: _isDark() ? _kDisabledDark : _kDisabledLight);
      case StepState.error:
        return textTheme.bodyLarge!.copyWith(color: _isDark() ? _kErrorDark : _kErrorLight);
    }
  }

  Widget _buildHeaderText(int index) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        AnimatedDefaultTextStyle(
          style: _titleStyle(index),
          duration: kThemeAnimationDuration,
          curve: Curves.fastOutSlowIn,
          child: widget.steps[index].title,
        ),
        if (widget.steps[index].subtitle != null)
          Padding(
            padding: const EdgeInsets.only(top: 2.0),
            child: AnimatedDefaultTextStyle(
              style: _subtitleStyle(index),
              duration: kThemeAnimationDuration,
              curve: Curves.fastOutSlowIn,
              child: widget.steps[index].subtitle!,
            ),
          ),
      ],
    );
  }

  Widget _buildLabelText(int index) {
    if (widget.steps[index].label != null) {
      return AnimatedDefaultTextStyle(
        style: _labelStyle(index),
        duration: kThemeAnimationDuration,
        child: widget.steps[index].label!,
      );
    }
    return const SizedBox.shrink();
  }

  Widget _buildVerticalHeader(int index) {
    final bool isActive = widget.steps[index].isActive;
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 24.0),
      child: Row(
        children: <Widget>[
          Column(
            children: <Widget>[
              // Line parts are always added in order for the ink splash to
              // flood the tips of the connector lines.
              _buildLine(!_isFirst(index), isActive),
              _buildIcon(index),
              _buildLine(!_isLast(index), isActive),
            ],
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsetsDirectional.only(start: 12.0),
              child: _buildHeaderText(index),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildVerticalBody(int index) {
    final double? marginLeft = _stepIconMargin?.resolve(TextDirection.ltr).left;
    final double? marginRight = _stepIconMargin?.resolve(TextDirection.ltr).right;
    final double? additionalMarginLeft = marginLeft != null ? marginLeft / 2.0 : null;
    final double? additionalMarginRight = marginRight != null ? marginRight / 2.0 : null;

    return Stack(
      children: <Widget>[
        PositionedDirectional(
          // When use margin affects the left or right side of the child, we
          // need to add half of the margin to the start or end of the child
          // respectively to get the correct positioning.
          start: 24.0 + (additionalMarginLeft ?? 0.0) + (additionalMarginRight ?? 0.0),
          top: 0.0,
          bottom: 0.0,
          width: _stepIconWidth ?? _kStepSize,
          child: Center(
            // The line is drawn from the center of the circle vertically until
            // it reaches the bottom and then horizontally to the edge of the
            // stepper.
            child: SizedBox(
              width: !_isLast(index) ? (widget.connectorThickness ?? 1.0) : 0.0,
              height: double.infinity,
              child: ColoredBox(color: _connectorColor(widget.steps[index].isActive)),
            ),
          ),
        ),
        AnimatedCrossFade(
          firstChild: const SizedBox(width: double.infinity, height: 0),
          secondChild: Padding(
            padding: EdgeInsetsDirectional.only(
              // Adjust [controlsBuilder] padding so that the content is
              // centered vertically.
              start: 60.0 + (marginLeft ?? 0.0),
              end: 24.0,
              bottom: 24.0,
            ),
            child: Column(
              children: <Widget>[
                ClipRect(clipBehavior: widget.clipBehavior, child: widget.steps[index].content),
                _buildVerticalControls(index),
              ],
            ),
          ),
          firstCurve: const Interval(0.0, 0.6, curve: Curves.fastOutSlowIn),
          secondCurve: const Interval(0.4, 1.0, curve: Curves.fastOutSlowIn),
          sizeCurve: Curves.fastOutSlowIn,
          crossFadeState: _isCurrent(index) ? CrossFadeState.showSecond : CrossFadeState.showFirst,
          duration: kThemeAnimationDuration,
        ),
      ],
    );
  }

  Widget _buildVertical() {
    return ListView(
      controller: widget.controller,
      shrinkWrap: true,
      physics: widget.physics,
      children: <Widget>[
        for (int i = 0; i < widget.steps.length; i += 1)
          Column(
            key: _keys[i],
            children: <Widget>[
              InkWell(
                onTap:
                    widget.steps[i].state != StepState.disabled
                        ? () {
                          // In the vertical case we need to scroll to the newly tapped
                          // step.
                          Scrollable.ensureVisible(
                            _keys[i].currentContext!,
                            curve: Curves.fastOutSlowIn,
                            duration: kThemeAnimationDuration,
                          );

                          widget.onStepTapped?.call(i);
                        }
                        : null,
                canRequestFocus: widget.steps[i].state != StepState.disabled,
                child: _buildVerticalHeader(i),
              ),
              _buildVerticalBody(i),
            ],
          ),
      ],
    );
  }

  Widget _buildHorizontal() {
    final List<Widget> children = <Widget>[
      for (int i = 0; i < widget.steps.length; i += 1) ...<Widget>[
        InkResponse(
          onTap:
              widget.steps[i].state != StepState.disabled
                  ? () {
                    widget.onStepTapped?.call(i);
                  }
                  : null,
          canRequestFocus: widget.steps[i].state != StepState.disabled,
          child: Row(
            children: <Widget>[
              SizedBox(
                height: _isLabel() ? 104.0 : 72.0,
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: <Widget>[
                    if (widget.steps[i].label != null) const SizedBox(height: 24.0),
                    Center(child: _buildIcon(i)),
                    if (widget.steps[i].label != null)
                      SizedBox(height: 24.0, child: _buildLabelText(i)),
                  ],
                ),
              ),
              Padding(
                padding: _stepIconMargin ?? const EdgeInsetsDirectional.only(start: 12.0),
                child: _buildHeaderText(i),
              ),
            ],
          ),
        ),
        if (!_isLast(i))
          Expanded(
            child: Padding(
              key: Key('line$i'),
              padding: _stepIconMargin ?? const EdgeInsets.symmetric(horizontal: 8.0),
              child: SizedBox(
                height:
                    widget.steps[i].stepStyle?.connectorThickness ??
                    widget.connectorThickness ??
                    1.0,
                child: ColoredBox(
                  color:
                      widget.steps[i].stepStyle?.connectorColor ??
                      _connectorColor(widget.steps[i].isActive),
                ),
              ),
            ),
          ),
      ],
    ];

    final List<Widget> stepPanels = <Widget>[];
    for (int i = 0; i < widget.steps.length; i += 1) {
      stepPanels.add(
        Visibility(
          maintainState: true,
          visible: i == widget.currentStep,
          child: ClipRect(clipBehavior: widget.clipBehavior, child: widget.steps[i].content),
        ),
      );
    }

    return Column(
      children: <Widget>[
        Material(
          elevation: widget.elevation ?? 2,
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 24.0),
            child: SizedBox(
              height: _stepIconHeight != null ? _stepIconHeight! * _heightFactor : null,
              child: Row(children: children),
            ),
          ),
        ),
        Expanded(
          child: ListView(
            controller: widget.controller,
            physics: widget.physics,
            padding: const EdgeInsets.all(24.0),
            children: <Widget>[
              AnimatedSize(
                curve: Curves.fastOutSlowIn,
                duration: kThemeAnimationDuration,
                child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: stepPanels),
              ),
              _buildVerticalControls(widget.currentStep),
            ],
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    assert(debugCheckHasMaterialLocalizations(context));
    assert(() {
      if (context.findAncestorWidgetOfExactType<Stepper>() != null) {
        throw FlutterError(
          'Steppers must not be nested.\n'
          'The material specification advises that one should avoid embedding '
          'steppers within steppers. '
          'https://material.io/archive/guidelines/components/steppers.html#steppers-usage',
        );
      }
      return true;
    }());
    return switch (widget.type) {
      StepperType.vertical => _buildVertical(),
      StepperType.horizontal => _buildHorizontal(),
    };
  }
}

// Paints a triangle whose base is the bottom of the bounding rectangle and its
// top vertex the middle of its top.
class _TrianglePainter extends CustomPainter {
  _TrianglePainter({required this.color});

  final Color color;

  @override
  bool hitTest(Offset point) => true; // Hitting the rectangle is fine enough.

  @override
  bool shouldRepaint(_TrianglePainter oldPainter) {
    return oldPainter.color != color;
  }

  @override
  void paint(Canvas canvas, Size size) {
    final double base = size.width;
    final double halfBase = size.width / 2.0;
    final double height = size.height;
    final List<Offset> points = <Offset>[
      Offset(0.0, height),
      Offset(base, height),
      Offset(halfBase, 0.0),
    ];

    canvas.drawPath(Path()..addPolygon(points, true), Paint()..color = color);
  }
}

/// This class is used to override the default visual properties of [Step] widgets within a [Stepper].
///
/// To customize the appearance of a [Step] create an instance of this class with non-null parameters
/// for the step properties whose default value you want to override.
///
/// Example usage:
/// ```dart
/// Step(
///   title: const Text('Step 1'),
///   content: const Text('Content for Step 1'),
///   stepStyle: StepStyle(
///     color: Colors.blue,
///     errorColor: Colors.red,
///     border: Border.all(color: Colors.grey),
///     boxShadow: const BoxShadow(blurRadius: 3.0, color: Colors.black26),
///     gradient: const LinearGradient(colors: <Color>[Colors.red, Colors.blue]),
///     indexStyle: const TextStyle(color: Colors.white),
///   ),
/// )
/// ```
///
/// {@tool dartpad}
/// An example that uses [StepStyle] to customize the appearance of each [Step] in a [Stepper].
///
/// ** See code in examples/api/lib/material/stepper/step_style.0.dart **
/// {@end-tool}

@immutable
class StepStyle with Diagnosticable {
  /// Constructs a [StepStyle].
  const StepStyle({
    this.color,
    this.errorColor,
    this.connectorColor,
    this.connectorThickness,
    this.border,
    this.boxShadow,
    this.gradient,
    this.indexStyle,
  });

  /// Overrides the default color of the circle in the step.
  final Color? color;

  /// Overrides the default color of the error indicator in the step.
  final Color? errorColor;

  /// Overrides the default color of the connector line between two steps.
  ///
  /// This property only applies when [Stepper.type] is [StepperType.horizontal].
  final Color? connectorColor;

  /// Overrides the default thickness of the connector line between two steps.
  ///
  /// This property only applies when [Stepper.type] is [StepperType.horizontal].
  final double? connectorThickness;

  /// Add a border around the step.
  ///
  /// Will be applied to the circle in the step.
  final BoxBorder? border;

  /// Add a shadow around the step.
  final BoxShadow? boxShadow;

  /// Add a gradient around the step.
  ///
  /// If [gradient] is specified, [color] will be ignored.
  final Gradient? gradient;

  /// Overrides the default style of the index in the step.
  final TextStyle? indexStyle;

  /// Returns a copy of this ButtonStyle with the given fields replaced with
  /// the new values.
  StepStyle copyWith({
    Color? color,
    Color? errorColor,
    Color? connectorColor,
    double? connectorThickness,
    BoxBorder? border,
    BoxShadow? boxShadow,
    Gradient? gradient,
    TextStyle? indexStyle,
  }) {
    return StepStyle(
      color: color ?? this.color,
      errorColor: errorColor ?? this.errorColor,
      connectorColor: connectorColor ?? this.connectorColor,
      connectorThickness: connectorThickness ?? this.connectorThickness,
      border: border ?? this.border,
      boxShadow: boxShadow ?? this.boxShadow,
      gradient: gradient ?? this.gradient,
      indexStyle: indexStyle ?? this.indexStyle,
    );
  }

  /// Returns a copy of this StepStyle where the non-null fields in [stepStyle]
  /// have replaced the corresponding null fields in this StepStyle.
  ///
  /// In other words, [stepStyle] is used to fill in unspecified (null) fields
  /// this StepStyle.
  StepStyle merge(StepStyle? stepStyle) {
    if (stepStyle == null) {
      return this;
    }
    return copyWith(
      color: stepStyle.color,
      errorColor: stepStyle.errorColor,
      connectorColor: stepStyle.connectorColor,
      connectorThickness: stepStyle.connectorThickness,
      border: stepStyle.border,
      boxShadow: stepStyle.boxShadow,
      gradient: stepStyle.gradient,
      indexStyle: stepStyle.indexStyle,
    );
  }

  @override
  int get hashCode {
    return Object.hash(
      color,
      errorColor,
      connectorColor,
      connectorThickness,
      border,
      boxShadow,
      gradient,
      indexStyle,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is StepStyle &&
        other.color == color &&
        other.errorColor == errorColor &&
        other.connectorColor == connectorColor &&
        other.connectorThickness == connectorThickness &&
        other.border == border &&
        other.boxShadow == boxShadow &&
        other.gradient == gradient &&
        other.indexStyle == indexStyle;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    final ThemeData theme = ThemeData.fallback();
    final TextTheme defaultTextTheme = theme.textTheme;
    properties.add(ColorProperty('color', color, defaultValue: null));
    properties.add(ColorProperty('errorColor', errorColor, defaultValue: null));
    properties.add(ColorProperty('connectorColor', connectorColor, defaultValue: null));
    properties.add(DoubleProperty('connectorThickness', connectorThickness, defaultValue: null));
    properties.add(DiagnosticsProperty<BoxBorder>('border', border, defaultValue: null));
    properties.add(DiagnosticsProperty<BoxShadow>('boxShadow', boxShadow, defaultValue: null));
    properties.add(DiagnosticsProperty<Gradient>('gradient', gradient, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'indexStyle',
        indexStyle,
        defaultValue: defaultTextTheme.bodyLarge,
      ),
    );
  }
}
