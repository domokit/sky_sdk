// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'card.dart';
/// @docImport 'color_scheme.dart';
/// @docImport 'colors.dart';
/// @docImport 'ink_decoration.dart';
/// @docImport 'ink_highlight.dart';
/// @docImport 'ink_splash.dart';
/// @docImport 'ink_well.dart';
/// @docImport 'list_tile.dart';
/// @docImport 'material_button.dart';
/// @docImport 'mergeable_material.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'constants.dart';
import 'elevation_overlay.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Signature for the callback used by ink effects to obtain the rectangle for the effect.
///
/// Used by [InkHighlight] and [InkSplash], for example.
typedef RectCallback = Rect Function();

/// The various kinds of material in Material Design. Used to
/// configure the default behavior of [Material] widgets.
///
/// See also:
///
///  * [Material], in particular [Material.type].
///  * [kMaterialEdges]
enum MaterialType {
  /// Rectangle using default theme canvas color.
  canvas,

  /// Rounded edges, card theme color.
  card,

  /// A circle, no color by default (used for floating action buttons).
  circle,

  /// Rounded edges, no color by default (used for [MaterialButton] buttons).
  button,

  /// A transparent piece of material that draws ink splashes and highlights.
  ///
  /// While the material metaphor describes child widgets as printed on the
  /// material itself and do not hide ink effects, in practice the [Material]
  /// widget draws child widgets on top of the ink effects.
  /// A [Material] with type transparency can be placed on top of opaque widgets
  /// to show ink effects on top of them.
  ///
  /// Prefer using the [Ink] widget for showing ink effects on top of opaque
  /// widgets.
  transparency,
}

/// The border radii used by the various kinds of material in Material Design.
///
/// See also:
///
///  * [MaterialType]
///  * [Material]
const Map<MaterialType, BorderRadius?> kMaterialEdges = <MaterialType, BorderRadius?>{
  MaterialType.canvas: null,
  MaterialType.card: BorderRadius.all(Radius.circular(2.0)),
  MaterialType.circle: null,
  MaterialType.button: BorderRadius.all(Radius.circular(2.0)),
  MaterialType.transparency: null,
};

/// An interface for creating [InkSplash]s and [InkHighlight]s on a [Material].
///
/// Typically obtained via [Material.of].
//
// TODO(nate-thegrate): deprecate Material-specific symbols once Splash
// fully replaces them.
// The Material widget will be refactored to use a SplashBox in its build method.
// https://github.com/flutter/flutter/issues/150139
typedef MaterialInkController = SplashController;

/// Used by [InteractiveInkFeatureFactory] to create [Splash]es.
///
/// Splash effects can be painted on an ancestor [Material] widget by obtaining
/// an instance of this controller via [Material.of] and calling [addSplash].
extension type SplashController._(_RenderSplashes _renderSplashes) implements Object {
  /// The [TickerProvider] used by this controller.
  ///
  /// [Splash]es added to this controller with [addSplash]
  /// use this vsync to drive their animations.
  TickerProvider get vsync => _renderSplashes.vsync;

  /// Suggests a color for the surface where splashes are painted.
  ///
  /// This color does not directly determine the appearance of the splash;
  /// it serves as a reference value that descendants can optionally access
  /// via `Material.of(context).color`.
  ///
  /// Unlike standard [InheritedWidget] patterns, changes to this
  /// color will **not** trigger automatic updates through `of(context)`.
  ///
  /// Additionally, if this color is non-null, the associated [RenderBox]
  /// will absorb hit tests. However, this behavior usually has minimal impact
  /// since splash controllers are often nested within widgets that are
  /// either fully opaque, or have opaque children.
  Color? get color => _renderSplashes.color;

  /// A list containing each [Splash] effect managed by this controller.
  ///
  /// Accessed during tests to verify that splashes are properly
  /// being added & removed.
  @protected
  @visibleForTesting
  List<Splash>? get splashes => _renderSplashes._splashes;

  /// Add an [InkFeature], such as an [InkSplash] or an [InkHighlight].
  ///
  /// The ink feature will paint as part of this controller.
  //
  // TODO(nate-thegrate): deprecate this method once Splash is officially introduced!
  // https://github.com/flutter/flutter/issues/150139
  void addInkFeature(InkFeature feature) => addSplash(feature);

  /// Adds a [Splash] to the collection of [splashes] painted by this controller.
  void addSplash(Splash splash) {
    assert(!splash._debugDisposed);
    assert(splash.controller == this);
    final List<Splash> splashes = _renderSplashes._splashes ??= <Splash>[];
    assert(!splashes.contains(splash));
    splashes.add(splash);
    markNeedsPaint();
  }

  void _removeSplash(Splash splash) {
    assert(splashes != null);
    splashes!.remove(splash);
    markNeedsPaint();
  }

  /// Notifies the controller that one of its ink features needs to repaint.
  void markNeedsPaint() => _renderSplashes.markNeedsPaint();
}

/// A piece of material.
///
/// The Material widget is responsible for:
///
/// 1. Clipping: If [clipBehavior] is not [Clip.none], Material clips its widget
///    sub-tree to the shape specified by [shape], [type], and [borderRadius].
///    By default, [clipBehavior] is [Clip.none] for performance considerations.
///    See [Ink] for an example of how this affects clipping [Ink] widgets.
/// 2. Elevation: Material elevates its widget sub-tree on the Z axis by
///    [elevation] pixels, and draws the appropriate shadow.
/// 3. Ink effects: Material shows ink effects implemented by [InkFeature]s
///    like [InkSplash] and [InkHighlight] below its children.
///
/// ## The Material Metaphor
///
/// Material is the central metaphor in Material Design. Each piece of material
/// exists at a given elevation, which influences how that piece of material
/// visually relates to other pieces of material and how that material casts
/// shadows.
///
/// Most user interface elements are either conceptually printed on a piece of
/// material or themselves made of material. Material reacts to user input using
/// [InkSplash] and [InkHighlight] effects. To trigger a reaction on the
/// material, use a [MaterialInkController] obtained via [Material.of].
///
/// In general, the features of a [Material] should not change over time (e.g. a
/// [Material] should not change its [color], [shadowColor] or [type]).
/// Changes to [elevation], [shadowColor] and [surfaceTintColor] are animated
/// for [animationDuration]. Changes to [shape] are animated if [type] is
/// not [MaterialType.transparency] and [ShapeBorder.lerp] between the previous
/// and next [shape] values is supported. Shape changes are also animated
/// for [animationDuration].
///
/// ## Shape
///
/// The shape for material is determined by [shape], [type], and [borderRadius].
///
///  - If [shape] is non null, it determines the shape.
///  - If [shape] is null and [borderRadius] is non null, the shape is a
///    rounded rectangle, with corners specified by [borderRadius].
///  - If [shape] and [borderRadius] are null, [type] determines the
///    shape as follows:
///    - [MaterialType.canvas]: the default material shape is a rectangle.
///    - [MaterialType.card]: the default material shape is a rectangle with
///      rounded edges. The edge radii is specified by [kMaterialEdges].
///    - [MaterialType.circle]: the default material shape is a circle.
///    - [MaterialType.button]: the default material shape is a rectangle with
///      rounded edges. The edge radii is specified by [kMaterialEdges].
///    - [MaterialType.transparency]: the default material shape is a rectangle.
///
/// ## Border
///
/// If [shape] is not null, then its border will also be painted (if any).
///
/// ## Layout change notifications
///
/// If the layout changes (e.g. because there's a list on the material, and it's
/// been scrolled), a [LayoutChangedNotification] must be dispatched at the
/// relevant subtree. This in particular means that transitions (e.g.
/// [SlideTransition]) should not be placed inside [Material] widgets so as to
/// move subtrees that contain [InkResponse]s, [InkWell]s, [Ink]s, or other
/// widgets that use the [InkFeature] mechanism. Otherwise, in-progress ink
/// features (e.g., ink splashes and ink highlights) won't move to account for
/// the new layout.
///
/// ## Painting over the material
///
/// Material widgets will often trigger reactions on their nearest material
/// ancestor. For example, [ListTile.hoverColor] triggers a reaction on the
/// tile's material when a pointer is hovering over it. These reactions will be
/// obscured if any widget in between them and the material paints in such a
/// way as to obscure the material (such as setting a [BoxDecoration.color] on
/// a [DecoratedBox]). To avoid this behavior, use [InkDecoration] to decorate
/// the material itself.
///
/// See also:
///
///  * [MergeableMaterial], a piece of material that can split and re-merge.
///  * [Card], a wrapper for a [Material] of [type] [MaterialType.card].
///  * <https://material.io/design/>
///  * <https://m3.material.io/styles/color/the-color-system/color-roles>
class Material extends StatefulWidget {
  /// Creates a piece of material.
  ///
  /// The [elevation] must be non-negative.
  ///
  /// If a [shape] is specified, then the [borderRadius] property must be
  /// null and the [type] property must not be [MaterialType.circle]. If the
  /// [borderRadius] is specified, then the [type] property must not be
  /// [MaterialType.circle]. In both cases, these restrictions are intended to
  /// catch likely errors.
  const Material({
    super.key,
    this.type = MaterialType.canvas,
    this.elevation = 0.0,
    this.color,
    this.shadowColor,
    this.surfaceTintColor,
    this.textStyle,
    this.borderRadius,
    this.shape,
    this.borderOnForeground = true,
    this.clipBehavior = Clip.none,
    this.animationDuration = kThemeChangeDuration,
    this.child,
  }) : assert(elevation >= 0.0),
       assert(!(shape != null && borderRadius != null)),
       assert(!(identical(type, MaterialType.circle) && (borderRadius != null || shape != null)));

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The kind of material to show (e.g., card or canvas). This
  /// affects the shape of the widget, the roundness of its corners if
  /// the shape is rectangular, and the default color.
  final MaterialType type;

  /// {@template flutter.material.material.elevation}
  /// The z-coordinate at which to place this material relative to its parent.
  ///
  /// This controls the size of the shadow below the material and the opacity
  /// of the elevation overlay color if it is applied.
  ///
  /// If this is non-zero, the contents of the material are clipped, because the
  /// widget conceptually defines an independent printed piece of material.
  ///
  /// Defaults to 0. Changing this value will cause the shadow and the elevation
  /// overlay or surface tint to animate over [Material.animationDuration].
  ///
  /// The value is non-negative.
  ///
  /// See also:
  ///
  ///  * [ThemeData.useMaterial3] which defines whether a surface tint or
  ///    elevation overlay is used to indicate elevation.
  ///  * [ThemeData.applyElevationOverlayColor] which controls the whether
  ///    an overlay color will be applied to indicate elevation.
  ///  * [Material.color] which may have an elevation overlay applied.
  ///  * [Material.shadowColor] which will be used for the color of a drop shadow.
  ///  * [Material.surfaceTintColor] which will be used as the overlay tint to
  ///    show elevation.
  /// {@endtemplate}
  final double elevation;

  /// The color to paint the material.
  ///
  /// Must be opaque. To create a transparent piece of material, use
  /// [MaterialType.transparency].
  ///
  /// If [ThemeData.useMaterial3] is true then an optional [surfaceTintColor]
  /// overlay may be applied on top of this color to indicate elevation.
  ///
  /// If [ThemeData.useMaterial3] is false and [ThemeData.applyElevationOverlayColor]
  /// is true and [ThemeData.brightness] is [Brightness.dark] then a
  /// semi-transparent overlay color will be composited on top of this
  /// color to indicate the elevation. This is no longer needed for Material
  /// Design 3, which uses [surfaceTintColor].
  ///
  /// By default, the color is derived from the [type] of material.
  final Color? color;

  /// The color to paint the shadow below the material.
  ///
  /// {@template flutter.material.material.shadowColor}
  /// If null and [ThemeData.useMaterial3] is true then [ThemeData]'s
  /// [ColorScheme.shadow] will be used. If [ThemeData.useMaterial3] is false
  /// then [ThemeData.shadowColor] will be used.
  ///
  /// To remove the drop shadow when [elevation] is greater than 0, set
  /// [shadowColor] to [Colors.transparent].
  ///
  /// See also:
  ///  * [ThemeData.useMaterial3], which determines the default value for this
  ///    property if it is null.
  ///  * [ThemeData.applyElevationOverlayColor], which turns elevation overlay
  /// on or off for dark themes.
  /// {@endtemplate}
  final Color? shadowColor;

  /// The color of the surface tint overlay applied to the material color
  /// to indicate elevation.
  ///
  /// {@template flutter.material.material.surfaceTintColor}
  /// Material Design 3 introduced a new way for some components to indicate
  /// their elevation by using a surface tint color overlay on top of the
  /// base material [color]. This overlay is painted with an opacity that is
  /// related to the [elevation] of the material.
  ///
  /// If [ThemeData.useMaterial3] is false, then this property is not used.
  ///
  /// If [ThemeData.useMaterial3] is true and [surfaceTintColor] is not null and
  /// not [Colors.transparent], then it will be used to overlay the base [color]
  /// with an opacity based on the [elevation].
  ///
  /// Otherwise, no surface tint will be applied.
  ///
  /// See also:
  ///
  ///   * [ThemeData.useMaterial3], which turns this feature on.
  ///   * [ElevationOverlay.applySurfaceTint], which is used to implement the
  ///     tint.
  ///   * https://m3.material.io/styles/color/the-color-system/color-roles
  ///     which specifies how the overlay is applied.
  /// {@endtemplate}
  final Color? surfaceTintColor;

  /// The typographical style to use for text within this material.
  final TextStyle? textStyle;

  /// Defines the material's shape as well its shadow.
  ///
  /// {@template flutter.material.material.shape}
  /// If shape is non null, the [borderRadius] is ignored and the material's
  /// clip boundary and shadow are defined by the shape.
  ///
  /// A shadow is only displayed if the [elevation] is greater than
  /// zero.
  /// {@endtemplate}
  final ShapeBorder? shape;

  /// Whether to paint the [shape] border in front of the [child].
  ///
  /// The default value is true.
  /// If false, the border will be painted behind the [child].
  final bool borderOnForeground;

  /// {@template flutter.material.Material.clipBehavior}
  /// The content will be clipped (or not) according to this option.
  ///
  /// See the enum [Clip] for details of all possible options and their common
  /// use cases.
  /// {@endtemplate}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// Defines the duration of animated changes for [shape], [elevation],
  /// [shadowColor], [surfaceTintColor] and the elevation overlay if it is applied.
  ///
  /// The default value is [kThemeChangeDuration].
  final Duration animationDuration;

  /// If non-null, the corners of this box are rounded by this
  /// [BorderRadiusGeometry] value.
  ///
  /// Otherwise, the corners specified for the current [type] of material are
  /// used.
  ///
  /// If [shape] is non null then the border radius is ignored.
  ///
  /// Must be null if [type] is [MaterialType.circle].
  final BorderRadiusGeometry? borderRadius;

  /// The ink controller from the closest instance of this class that
  /// encloses the given context within the closest [LookupBoundary].
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// MaterialInkController? inkController = Material.maybeOf(context);
  /// ```
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// See also:
  ///
  /// * [Material.of], which is similar to this method, but asserts if
  ///   no [Material] ancestor is found.
  static MaterialInkController? maybeOf(BuildContext context) {
    return LookupBoundary.findAncestorRenderObjectOfType<_RenderSplashes>(context)?.asController;
  }

  /// The ink controller from the closest instance of [Material] that encloses
  /// the given context within the closest [LookupBoundary].
  ///
  /// If no [Material] widget ancestor can be found then this method will assert
  /// in debug mode, and throw an exception in release mode.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// MaterialInkController inkController = Material.of(context);
  /// ```
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// See also:
  ///
  /// * [Material.maybeOf], which is similar to this method, but returns null if
  ///   no [Material] ancestor is found.
  static MaterialInkController of(BuildContext context) {
    final MaterialInkController? controller = maybeOf(context);
    assert(() {
      if (controller == null) {
        if (LookupBoundary.debugIsHidingAncestorRenderObjectOfType<_RenderSplashes>(context)) {
          throw FlutterError(
            'Material.of() was called with a context that does not have access to a Material widget.\n'
            'The context provided to Material.of() does have a Material widget ancestor, but it is '
            'hidden by a LookupBoundary. This can happen because you are using a widget that looks '
            'for a Material ancestor, but no such ancestor exists within the closest LookupBoundary.\n'
            'The context used was:\n'
            '  $context',
          );
        }
        throw FlutterError(
          'Material.of() was called with a context that does not contain a Material widget.\n'
          'No Material widget ancestor could be found starting from the context that was passed to '
          'Material.of(). This can happen because you are using a widget that looks for a Material '
          'ancestor, but no such ancestor exists.\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return controller!;
  }

  @override
  State<Material> createState() => _MaterialState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<MaterialType>('type', type));
    properties.add(DoubleProperty('elevation', elevation, defaultValue: 0.0));
    properties.add(ColorProperty('color', color, defaultValue: null));
    properties.add(ColorProperty('shadowColor', shadowColor, defaultValue: null));
    properties.add(ColorProperty('surfaceTintColor', surfaceTintColor, defaultValue: null));
    textStyle?.debugFillProperties(properties, prefix: 'textStyle.');
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(
      DiagnosticsProperty<bool>('borderOnForeground', borderOnForeground, defaultValue: true),
    );
    properties.add(
      DiagnosticsProperty<BorderRadiusGeometry>('borderRadius', borderRadius, defaultValue: null),
    );
  }

  /// The default radius of an ink splash in logical pixels.
  static const double defaultSplashRadius = 35.0;
}

class _MaterialState extends State<Material> with TickerProviderStateMixin {
  final GlobalKey _splashRenderer = GlobalKey(debugLabel: 'splash renderer');

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final Color? backgroundColor =
        widget.color ??
        switch (widget.type) {
          MaterialType.canvas => theme.canvasColor,
          MaterialType.card => theme.cardColor,
          MaterialType.button || MaterialType.circle || MaterialType.transparency => null,
        };
    final Color modelShadowColor =
        widget.shadowColor ?? (theme.useMaterial3 ? theme.colorScheme.shadow : theme.shadowColor);
    assert(
      backgroundColor != null || widget.type == MaterialType.transparency,
      'If Material type is not MaterialType.transparency, a color must '
      'either be passed in through the `color` property, or be defined '
      'in the theme (ex. canvasColor != null if type is set to '
      'MaterialType.canvas)',
    );

    Widget? contents = widget.child;
    if (contents != null) {
      contents = AnimatedDefaultTextStyle(
        style: widget.textStyle ?? Theme.of(context).textTheme.bodyMedium!,
        duration: widget.animationDuration,
        child: contents,
      );
    }
    contents = NotificationListener<LayoutChangedNotification>(
      onNotification: (LayoutChangedNotification notification) {
        final _RenderSplashes renderer =
            _splashRenderer.currentContext!.findRenderObject()! as _RenderSplashes;
        renderer._didChangeLayout();
        return false;
      },
      child: _Splashes(
        key: _splashRenderer,
        color: widget.type == MaterialType.transparency ? null : backgroundColor,
        vsync: this,
        child: contents,
      ),
    );

    ShapeBorder? shape =
        widget.borderRadius != null
            ? RoundedRectangleBorder(borderRadius: widget.borderRadius!)
            : widget.shape;

    // PhysicalModel has a temporary workaround for a performance issue that
    // speeds up rectangular non transparent material (the workaround is to
    // skip the call to ui.Canvas.saveLayer if the border radius is 0).
    // Until the saveLayer performance issue is resolved, we're keeping this
    // special case here for canvas material type that is using the default
    // shape (rectangle). We could go down this fast path for explicitly
    // specified rectangles (e.g shape RoundedRectangleBorder with radius 0, but
    // we choose not to as we want the change from the fast-path to the
    // slow-path to be noticeable in the construction site of Material.
    if (widget.type == MaterialType.canvas && shape == null) {
      final Color color =
          theme.useMaterial3
              ? ElevationOverlay.applySurfaceTint(
                backgroundColor!,
                widget.surfaceTintColor,
                widget.elevation,
              )
              : ElevationOverlay.applyOverlay(context, backgroundColor!, widget.elevation);

      return AnimatedPhysicalModel(
        curve: Curves.fastOutSlowIn,
        duration: widget.animationDuration,
        clipBehavior: widget.clipBehavior,
        elevation: widget.elevation,
        color: color,
        shadowColor: modelShadowColor,
        animateColor: false,
        child: contents,
      );
    }

    shape ??= switch (widget.type) {
      MaterialType.circle => const CircleBorder(),
      MaterialType.canvas || MaterialType.transparency => const RoundedRectangleBorder(),
      MaterialType.card || MaterialType.button => const RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(2.0)),
      ),
    };

    if (widget.type == MaterialType.transparency) {
      return ClipPath(
        clipper: ShapeBorderClipper(shape: shape, textDirection: Directionality.maybeOf(context)),
        clipBehavior: widget.clipBehavior,
        child: _ShapeBorderPaint(shape: shape, child: contents),
      );
    }

    return _MaterialInterior(
      curve: Curves.fastOutSlowIn,
      duration: widget.animationDuration,
      shape: shape,
      borderOnForeground: widget.borderOnForeground,
      clipBehavior: widget.clipBehavior,
      elevation: widget.elevation,
      color: backgroundColor!,
      shadowColor: modelShadowColor,
      surfaceTintColor: widget.surfaceTintColor,
      child: contents,
    );
  }
}

class _RenderSplashes extends RenderProxyBox {
  _RenderSplashes({required this.vsync, this.color}) : super(null);

  // This class should exist in a 1:1 relationship with a MaterialState object,
  // since there's no current support for dynamically changing the ticker
  // provider.
  final TickerProvider vsync;

  Color? color;

  List<Splash>? _splashes;

  SplashController get asController => SplashController._(this);

  void _didChangeLayout() {
    if (_splashes?.isNotEmpty ?? false) {
      markNeedsPaint();
    }
  }

  @override
  bool hitTestSelf(Offset position) => color != null;

  @override
  void paint(PaintingContext context, Offset offset) {
    final List<Splash>? splashes = _splashes;
    if (splashes != null && splashes.isNotEmpty) {
      final Canvas canvas = context.canvas;
      canvas.save();
      canvas.translate(offset.dx, offset.dy);
      canvas.clipRect(Offset.zero & size);
      for (final Splash inkFeature in splashes) {
        inkFeature._paint(canvas);
      }
      canvas.restore();
    }
    assert(splashes == _splashes);
    super.paint(context, offset);
  }
}

class _Splashes extends SingleChildRenderObjectWidget {
  const _Splashes({super.key, this.color, required this.vsync, super.child});

  // This widget must be owned by a MaterialState, which must be provided as the vsync.
  // This relationship must be 1:1 and cannot change for the lifetime of the MaterialState.

  final Color? color;

  final TickerProvider vsync;

  @override
  _RenderSplashes createRenderObject(BuildContext context) {
    return _RenderSplashes(color: color, vsync: vsync);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSplashes renderObject) {
    renderObject.color = color;
    assert(vsync == renderObject.vsync);
  }
}

/// An animation projected onto a [Material] in response to a user gesture.
///
/// See also:
///
/// * [SplashController], which is responsible for painting splashes.
/// * [Material], the only widget (as of now) that enables splash effects.
/// * [InteractiveInkFeatureFactory], a class that creates splashes through
///   a specific function signature.
/// * [InkResponse], which creates splashes using an [InteractiveInkFeatureFactory].
typedef Splash = InkFeature;

/// A visual reaction on a piece of [Material].
///
/// To add an ink feature to a piece of [Material], obtain the
/// [SplashController] via [Material.of] and call [SplashController.addSplash].
abstract class InkFeature {
  /// Initializes fields for subclasses.
  InkFeature({required this.controller, required this.referenceBox, this.onRemoved}) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/material.dart',
        className: '$InkFeature',
        object: this,
      );
    }
  }

  /// The [SplashController] associated with this [InkFeature].
  ///
  /// Typically used by subclasses to call
  /// [SplashController.markNeedsPaint] when they need to repaint.
  final SplashController controller;

  /// The render box whose visual position defines the frame of reference for this ink feature.
  final RenderBox referenceBox;

  /// Called when the ink feature is no longer visible on the material.
  final VoidCallback? onRemoved;

  bool _debugDisposed = false;

  /// Free up the resources associated with this ink feature.
  @mustCallSuper
  void dispose() {
    assert(!_debugDisposed);
    assert(() {
      _debugDisposed = true;
      return true;
    }());
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    controller._removeSplash(this);
    onRemoved?.call();
  }

  // Returns the paint transform that allows `fromRenderObject` to perform paint
  // in `toRenderObject`'s coordinate space.
  //
  // Returns null if either `fromRenderObject` or `toRenderObject` is not in the
  // same render tree, or either of them is in an offscreen subtree (see
  // RenderObject.paintsChild).
  static Matrix4? _getPaintTransform(RenderObject fromRenderObject, RenderObject toRenderObject) {
    // The paths to fromRenderObject and toRenderObject's common ancestor.
    final List<RenderObject> fromPath = <RenderObject>[fromRenderObject];
    final List<RenderObject> toPath = <RenderObject>[toRenderObject];

    RenderObject from = fromRenderObject;
    RenderObject to = toRenderObject;

    while (!identical(from, to)) {
      final int fromDepth = from.depth;
      final int toDepth = to.depth;

      if (fromDepth >= toDepth) {
        final RenderObject? fromParent = from.parent;
        // Return early if the 2 render objects are not in the same render tree,
        // or either of them is offscreen and thus won't get painted.
        if (fromParent is! RenderObject || !fromParent.paintsChild(from)) {
          return null;
        }
        fromPath.add(fromParent);
        from = fromParent;
      }

      if (fromDepth <= toDepth) {
        final RenderObject? toParent = to.parent;
        if (toParent is! RenderObject || !toParent.paintsChild(to)) {
          return null;
        }
        toPath.add(toParent);
        to = toParent;
      }
    }
    assert(identical(from, to));

    final Matrix4 transform = Matrix4.identity();
    final Matrix4 inverseTransform = Matrix4.identity();

    for (int index = toPath.length - 1; index > 0; index -= 1) {
      toPath[index].applyPaintTransform(toPath[index - 1], transform);
    }
    for (int index = fromPath.length - 1; index > 0; index -= 1) {
      fromPath[index].applyPaintTransform(fromPath[index - 1], inverseTransform);
    }

    final double det = inverseTransform.invert();
    return det != 0 ? (inverseTransform..multiply(transform)) : null;
  }

  void _paint(Canvas canvas) {
    assert(referenceBox.attached);
    assert(!_debugDisposed);
    // determine the transform that gets our coordinate system to be like theirs
    final Matrix4? transform = _getPaintTransform(controller._renderSplashes, referenceBox);
    if (transform != null) {
      paintFeature(canvas, transform);
    }
  }

  /// Override this method to paint the ink feature.
  ///
  /// The transform argument gives the coordinate conversion from the coordinate
  /// system of the canvas to the coordinate system of the [referenceBox].
  @protected
  void paintFeature(Canvas canvas, Matrix4 transform);

  @override
  String toString() => describeIdentity(this);
}

/// An interpolation between two [ShapeBorder]s.
///
/// This class specializes the interpolation of [Tween] to use [ShapeBorder.lerp].
class ShapeBorderTween extends Tween<ShapeBorder?> {
  /// Creates a [ShapeBorder] tween.
  ///
  /// the [begin] and [end] properties may be null; see [ShapeBorder.lerp] for
  /// the null handling semantics.
  ShapeBorderTween({super.begin, super.end});

  /// Returns the value this tween has at the given animation clock value.
  @override
  ShapeBorder? lerp(double t) {
    return ShapeBorder.lerp(begin, end, t);
  }
}

/// The interior of non-transparent material.
///
/// Animates [elevation], [shadowColor], and [shape].
class _MaterialInterior extends ImplicitlyAnimatedWidget {
  /// Creates a const instance of [_MaterialInterior].
  ///
  /// The [elevation] must be specified and greater than or equal to zero.
  const _MaterialInterior({
    required this.child,
    required this.shape,
    this.borderOnForeground = true,
    this.clipBehavior = Clip.none,
    required this.elevation,
    required this.color,
    required this.shadowColor,
    required this.surfaceTintColor,
    super.curve,
    required super.duration,
  }) : assert(elevation >= 0.0);

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The border of the widget.
  ///
  /// This border will be painted, and in addition the outer path of the border
  /// determines the physical shape.
  final ShapeBorder shape;

  /// Whether to paint the border in front of the child.
  ///
  /// The default value is true.
  /// If false, the border will be painted behind the child.
  final bool borderOnForeground;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// The target z-coordinate at which to place this physical object relative
  /// to its parent.
  ///
  /// The value is non-negative.
  final double elevation;

  /// The target background color.
  final Color color;

  /// The target shadow color.
  final Color shadowColor;

  /// The target surface tint color.
  final Color? surfaceTintColor;

  @override
  _MaterialInteriorState createState() => _MaterialInteriorState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(DiagnosticsProperty<ShapeBorder>('shape', shape));
    description.add(DoubleProperty('elevation', elevation));
    description.add(ColorProperty('color', color));
    description.add(ColorProperty('shadowColor', shadowColor));
  }
}

class _MaterialInteriorState extends AnimatedWidgetBaseState<_MaterialInterior> {
  Tween<double>? _elevation;
  ColorTween? _surfaceTintColor;
  ColorTween? _shadowColor;
  ShapeBorderTween? _border;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _elevation =
        visitor(
              _elevation,
              widget.elevation,
              (dynamic value) => Tween<double>(begin: value as double),
            )
            as Tween<double>?;
    _shadowColor =
        visitor(
              _shadowColor,
              widget.shadowColor,
              (dynamic value) => ColorTween(begin: value as Color),
            )
            as ColorTween?;
    _surfaceTintColor =
        widget.surfaceTintColor != null
            ? visitor(
                  _surfaceTintColor,
                  widget.surfaceTintColor,
                  (dynamic value) => ColorTween(begin: value as Color),
                )
                as ColorTween?
            : null;
    _border =
        visitor(
              _border,
              widget.shape,
              (dynamic value) => ShapeBorderTween(begin: value as ShapeBorder),
            )
            as ShapeBorderTween?;
  }

  @override
  Widget build(BuildContext context) {
    final ShapeBorder shape = _border!.evaluate(animation)!;
    final double elevation = _elevation!.evaluate(animation);
    final Color color =
        Theme.of(context).useMaterial3
            ? ElevationOverlay.applySurfaceTint(
              widget.color,
              _surfaceTintColor?.evaluate(animation),
              elevation,
            )
            : ElevationOverlay.applyOverlay(context, widget.color, elevation);
    final Color shadowColor = _shadowColor!.evaluate(animation)!;

    return PhysicalShape(
      clipper: ShapeBorderClipper(shape: shape, textDirection: Directionality.maybeOf(context)),
      clipBehavior: widget.clipBehavior,
      elevation: elevation,
      color: color,
      shadowColor: shadowColor,
      child: _ShapeBorderPaint(
        shape: shape,
        borderOnForeground: widget.borderOnForeground,
        child: widget.child,
      ),
    );
  }
}

class _ShapeBorderPaint extends StatelessWidget {
  const _ShapeBorderPaint({
    required this.child,
    required this.shape,
    this.borderOnForeground = true,
  });

  final Widget child;
  final ShapeBorder shape;
  final bool borderOnForeground;

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      painter:
          borderOnForeground ? null : _ShapeBorderPainter(shape, Directionality.maybeOf(context)),
      foregroundPainter:
          borderOnForeground ? _ShapeBorderPainter(shape, Directionality.maybeOf(context)) : null,
      child: child,
    );
  }
}

class _ShapeBorderPainter extends CustomPainter {
  _ShapeBorderPainter(this.border, this.textDirection);
  final ShapeBorder border;
  final TextDirection? textDirection;

  @override
  void paint(Canvas canvas, Size size) {
    border.paint(canvas, Offset.zero & size, textDirection: textDirection);
  }

  @override
  bool shouldRepaint(_ShapeBorderPainter oldDelegate) {
    return oldDelegate.border != border;
  }
}
