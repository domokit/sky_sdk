// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'adaptive_text_selection_toolbar.dart';
import 'back_button.dart';
import 'button_style.dart';
import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'divider.dart';
import 'divider_theme.dart';
import 'icon_button.dart';
import 'icons.dart';
import 'ink_well.dart';
import 'input_border.dart';
import 'input_decorator.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'search_bar_theme.dart';
import 'search_view_theme.dart';
import 'text_field.dart';
import 'text_theme.dart';
import 'theme.dart';
import 'theme_data.dart';

const int _kOpenViewMilliseconds = 600;
const Duration _kOpenViewDuration = Duration(milliseconds: _kOpenViewMilliseconds);
const Duration _kAnchorFadeDuration = Duration(milliseconds: 150);
const Curve _kViewFadeOnInterval = Interval(0.0, 1 / 2);
const Curve _kViewIconsFadeOnInterval = Interval(1 / 6, 2 / 6);
const Curve _kViewDividerFadeOnInterval = Interval(0.0, 1 / 6);
const Curve _kViewListFadeOnInterval = Interval(
  133 / _kOpenViewMilliseconds,
  233 / _kOpenViewMilliseconds,
);
const double _kDisableSearchBarOpacity = 0.38;

/// Signature for a function that creates a [Widget] which is used to open a search view.
///
/// The `controller` callback provided to [SearchAnchor.builder] can be used
/// to open the search view and control the editable field on the view.
typedef SearchAnchorChildBuilder =
    Widget Function(BuildContext context, SearchController controller);

/// Signature for a function that creates a [Widget] to build the suggestion list
/// based on the input in the search bar.
///
/// The `controller` callback provided to [SearchAnchor.suggestionsBuilder] can be used
/// to close the search view and control the editable field on the view.
typedef SuggestionsBuilder =
    FutureOr<Iterable<Widget>> Function(BuildContext context, SearchController controller);

/// Signature for a function that creates a [Widget] to layout the suggestion list.
///
/// Parameter `suggestions` is the content list that this function wants to lay out.
typedef ViewBuilder = Widget Function(Iterable<Widget> suggestions);

/// Manages a "search view" route that allows the user to select one of the
/// suggested completions for a search query.
///
/// The search view's route can either be shown by creating a [SearchController]
/// and then calling [SearchController.openView] or by tapping on an anchor.
/// When the anchor is tapped or [SearchController.openView] is called, the search view either
/// grows to a specific size, or grows to fill the entire screen. By default,
/// the search view only shows full screen on mobile platforms. Use [SearchAnchor.isFullScreen]
/// to override the default setting.
///
/// The search view is usually opened by a [SearchBar], an [IconButton] or an [Icon].
/// If [builder] returns an Icon, or any un-tappable widgets, we don't have
/// to explicitly call [SearchController.openView].
///
/// The search view route will be popped if the window size is changed and the
/// search view route is not in full-screen mode. However, if the search view route
/// is in full-screen mode, changing the window size, such as rotating a mobile
/// device from portrait mode to landscape mode, will not close the search view.
///
/// {@tool dartpad}
/// This example shows how to use an IconButton to open a search view in a [SearchAnchor].
/// It also shows how to use [SearchController] to open or close the search view route.
///
/// ** See code in examples/api/lib/material/search_anchor/search_anchor.2.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to set up a floating (or pinned) AppBar with a
/// [SearchAnchor] for a title.
///
/// ** See code in examples/api/lib/material/search_anchor/search_anchor.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to fetch the search suggestions from a remote API.
///
/// ** See code in examples/api/lib/material/search_anchor/search_anchor.3.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example demonstrates fetching the search suggestions asynchronously and
/// debouncing network calls.
///
/// ** See code in examples/api/lib/material/search_anchor/search_anchor.4.dart **
/// {@end-tool}
///
/// See also:
///
/// * [SearchBar], a widget that defines a search bar.
/// * [SearchBarTheme], a widget that overrides the default configuration of a search bar.
/// * [SearchViewTheme], a widget that overrides the default configuration of a search view.
class SearchAnchor extends StatefulWidget {
  /// Creates a const [SearchAnchor].
  ///
  /// The [builder] and [suggestionsBuilder] arguments are required.
  const SearchAnchor({
    super.key,
    this.isFullScreen,
    this.searchController,
    this.viewBuilder,
    this.viewLeading,
    this.viewTrailing,
    this.viewHintText,
    this.viewBackgroundColor,
    this.viewElevation,
    this.viewSurfaceTintColor,
    this.viewSide,
    this.viewShape,
    this.viewBarPadding,
    this.headerHeight,
    this.headerTextStyle,
    this.headerHintStyle,
    this.dividerColor,
    this.viewConstraints,
    this.viewPadding,
    this.shrinkWrap,
    this.textCapitalization,
    this.viewOnChanged,
    this.viewOnSubmitted,
    required this.builder,
    required this.suggestionsBuilder,
    this.textInputAction,
    this.keyboardType,
    this.enabled = true,
  });

  /// Create a [SearchAnchor] that has a [SearchBar] which opens a search view.
  ///
  /// All the barX parameters are used to customize the anchor. Similarly, all the
  /// viewX parameters are used to override the view's defaults.
  ///
  /// {@tool dartpad}
  /// This example shows how to use a [SearchAnchor.bar] which uses a default search
  /// bar to open a search view route.
  ///
  /// ** See code in examples/api/lib/material/search_anchor/search_anchor.0.dart **
  /// {@end-tool}
  factory SearchAnchor.bar({
    Widget? barLeading,
    Iterable<Widget>? barTrailing,
    String? barHintText,
    GestureTapCallback? onTap,
    ValueChanged<String>? onSubmitted,
    ValueChanged<String>? onChanged,
    MaterialStateProperty<double?>? barElevation,
    MaterialStateProperty<Color?>? barBackgroundColor,
    MaterialStateProperty<Color?>? barOverlayColor,
    MaterialStateProperty<BorderSide?>? barSide,
    MaterialStateProperty<OutlinedBorder?>? barShape,
    MaterialStateProperty<EdgeInsetsGeometry?>? barPadding,
    EdgeInsetsGeometry? viewBarPadding,
    MaterialStateProperty<TextStyle?>? barTextStyle,
    MaterialStateProperty<TextStyle?>? barHintStyle,
    ViewBuilder? viewBuilder,
    Widget? viewLeading,
    Iterable<Widget>? viewTrailing,
    String? viewHintText,
    Color? viewBackgroundColor,
    double? viewElevation,
    BorderSide? viewSide,
    OutlinedBorder? viewShape,
    double? viewHeaderHeight,
    TextStyle? viewHeaderTextStyle,
    TextStyle? viewHeaderHintStyle,
    Color? dividerColor,
    BoxConstraints? constraints,
    BoxConstraints? viewConstraints,
    EdgeInsetsGeometry? viewPadding,
    bool? shrinkWrap,
    bool? isFullScreen,
    SearchController searchController,
    TextCapitalization textCapitalization,
    required SuggestionsBuilder suggestionsBuilder,
    TextInputAction? textInputAction,
    TextInputType? keyboardType,
    EdgeInsets scrollPadding,
    EditableTextContextMenuBuilder contextMenuBuilder,
  }) = _SearchAnchorWithSearchBar;

  /// Whether the search view grows to fill the entire screen when the
  /// [SearchAnchor] is tapped.
  ///
  /// By default, the search view is full-screen on mobile devices. On other
  /// platforms, the search view only grows to a specific size that is determined
  /// by the anchor and the default size.
  final bool? isFullScreen;

  /// An optional controller that allows opening and closing of the search view from
  /// other widgets.
  ///
  /// If this is null, one internal search controller is created automatically
  /// and it is used to open the search view when the user taps on the anchor.
  final SearchController? searchController;

  /// Optional callback to obtain a widget to lay out the suggestion list of the
  /// search view.
  ///
  /// Default view uses a [ListView] with a vertical scroll direction.
  final ViewBuilder? viewBuilder;

  /// An optional widget to display before the text input field when the search
  /// view is open.
  ///
  /// Typically the [viewLeading] widget is an [Icon] or an [IconButton].
  ///
  /// Defaults to a back button which pops the view.
  final Widget? viewLeading;

  /// An optional widget list to display after the text input field when the search
  /// view is open.
  ///
  /// Typically the [viewTrailing] widget list only has one or two widgets.
  ///
  /// Defaults to an icon button which clears the text in the input field.
  final Iterable<Widget>? viewTrailing;

  /// Text that is displayed when the search bar's input field is empty.
  final String? viewHintText;

  /// The search view's background fill color.
  ///
  /// If null, the value of [SearchViewThemeData.backgroundColor] will be used.
  /// If this is also null, then the default value is [ColorScheme.surfaceContainerHigh].
  final Color? viewBackgroundColor;

  /// The elevation of the search view's [Material].
  ///
  /// If null, the value of [SearchViewThemeData.elevation] will be used. If this
  /// is also null, then default value is 6.0.
  final double? viewElevation;

  /// The surface tint color of the search view's [Material].
  ///
  /// This is not recommended for use. [Material 3 spec](https://m3.material.io/styles/color/the-color-system/color-roles)
  /// introduced a set of tone-based surfaces and surface containers in its [ColorScheme],
  /// which provide more flexibility. The intention is to eventually remove surface tint color from
  /// the framework.
  ///
  /// If null, the value of [SearchViewThemeData.surfaceTintColor] will be used.
  /// If this is also null, then the default value is [ColorScheme.surfaceTint].
  final Color? viewSurfaceTintColor;

  /// The color and weight of the search view's outline.
  ///
  /// This value is combined with [viewShape] to create a shape decorated
  /// with an outline. This will be ignored if the view is full-screen.
  ///
  /// If null, the value of [SearchViewThemeData.side] will be used. If this is
  /// also null, the search view doesn't have a side by default.
  final BorderSide? viewSide;

  /// The shape of the search view's underlying [Material].
  ///
  /// This shape is combined with [viewSide] to create a shape decorated
  /// with an outline.
  ///
  /// If null, the value of [SearchViewThemeData.shape] will be used.
  /// If this is also null, then the default value is a rectangle shape for full-screen
  /// mode and a [RoundedRectangleBorder] shape with a 28.0 radius otherwise.
  final OutlinedBorder? viewShape;

  /// The padding to use for the search view's search bar.
  ///
  /// If null, then the default value is 8.0 horizontally.
  final EdgeInsetsGeometry? viewBarPadding;

  /// The height of the search field on the search view.
  ///
  /// If null, the value of [SearchViewThemeData.headerHeight] will be used. If
  /// this is also null, the default value is 56.0.
  final double? headerHeight;

  /// The style to use for the text being edited on the search view.
  ///
  /// If null, defaults to the `bodyLarge` text style from the current [Theme].
  /// The default text color is [ColorScheme.onSurface].
  final TextStyle? headerTextStyle;

  /// The style to use for the [viewHintText] on the search view.
  ///
  /// If null, the value of [SearchViewThemeData.headerHintStyle] will be used.
  /// If this is also null, the value of [headerTextStyle] will be used. If this is also null,
  /// defaults to the `bodyLarge` text style from the current [Theme]. The default
  /// text color is [ColorScheme.onSurfaceVariant].
  final TextStyle? headerHintStyle;

  /// The color of the divider on the search view.
  ///
  /// If this property is null, then [SearchViewThemeData.dividerColor] is used.
  /// If that is also null, the default value is [ColorScheme.outline].
  final Color? dividerColor;

  /// Optional size constraints for the search view.
  ///
  /// By default, the search view has the same width as the anchor and is 2/3
  /// the height of the screen. If the width and height of the view are within
  /// the [viewConstraints], the view will show its default size. Otherwise,
  /// the size of the view will be constrained by this property.
  ///
  /// If null, the value of [SearchViewThemeData.constraints] will be used. If
  /// this is also null, then the constraints defaults to:
  /// ```dart
  /// const BoxConstraints(minWidth: 360.0, minHeight: 240.0)
  /// ```
  final BoxConstraints? viewConstraints;

  /// The padding to use for the search view.
  ///
  /// Has no effect if the search view is full-screen.
  ///
  /// If null, the value of [SearchViewThemeData.padding] will be used.
  final EdgeInsetsGeometry? viewPadding;

  /// Whether the search view should shrink-wrap its contents.
  ///
  /// Has no effect if the search view is full-screen.
  ///
  /// If null, the value of [SearchViewThemeData.shrinkWrap] will be used. If
  /// this is also null, then the default value is `false`.
  final bool? shrinkWrap;

  /// {@macro flutter.widgets.editableText.textCapitalization}
  final TextCapitalization? textCapitalization;

  /// Called each time the user modifies the search view's text field.
  ///
  /// See also:
  ///
  ///  * [viewOnSubmitted], which is called when the user indicates that they
  ///  are done editing the search view's text field.
  final ValueChanged<String>? viewOnChanged;

  /// Called when the user indicates that they are done editing the text in the
  /// text field of a search view. Typically this is called when the user presses
  /// the enter key.
  ///
  /// See also:
  ///
  /// * [viewOnChanged], which is called when the user modifies the text field
  /// of the search view.
  final ValueChanged<String>? viewOnSubmitted;

  /// Called to create a widget which can open a search view route when it is tapped.
  ///
  /// The widget returned by this builder is faded out when it is tapped.
  /// At the same time a search view route is faded in.
  final SearchAnchorChildBuilder builder;

  /// Called to get the suggestion list for the search view.
  ///
  /// By default, the list returned by this builder is laid out in a [ListView].
  /// To get a different layout, use [viewBuilder] to override.
  final SuggestionsBuilder suggestionsBuilder;

  /// {@macro flutter.widgets.TextField.textInputAction}
  final TextInputAction? textInputAction;

  /// The type of action button to use for the keyboard.
  ///
  /// Defaults to the default value specified in [TextField].
  final TextInputType? keyboardType;

  /// Whether or not this widget is currently interactive.
  ///
  /// When false, the widget will ignore taps and appear dimmed.
  ///
  /// Defaults to true.
  final bool enabled;

  @override
  State<SearchAnchor> createState() => _SearchAnchorState();
}

class _SearchAnchorState extends State<SearchAnchor> {
  Size? _screenSize;
  bool _anchorIsVisible = true;
  final GlobalKey _anchorKey = GlobalKey();
  bool get _viewIsOpen => !_anchorIsVisible;
  SearchController? _internalSearchController;
  SearchController get _searchController =>
      widget.searchController ?? (_internalSearchController ??= SearchController());
  _SearchViewRoute? _route;

  @override
  void initState() {
    super.initState();
    _searchController._attach(this);
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final Size updatedScreenSize = MediaQuery.of(context).size;
    if (_screenSize != null && _screenSize != updatedScreenSize) {
      if (_searchController.isOpen && !getShowFullScreenView()) {
        _closeView(null);
      }
    }
    _screenSize = updatedScreenSize;
  }

  @override
  void didUpdateWidget(SearchAnchor oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.searchController != widget.searchController) {
      oldWidget.searchController?._detach(this);
      _searchController._attach(this);
    }
  }

  @override
  void dispose() {
    widget.searchController?._detach(this);
    _internalSearchController?._detach(this);
    final bool usingExternalController = widget.searchController != null;
    if (_route?.navigator != null) {
      _route?._dismiss(disposeController: !usingExternalController);
      if (usingExternalController) {
        _internalSearchController?.dispose();
      }
    } else {
      _internalSearchController?.dispose();
    }
    super.dispose();
  }

  void _openView() {
    final NavigatorState navigator = Navigator.of(context);
    _route = _SearchViewRoute(
      viewOnChanged: widget.viewOnChanged,
      viewOnSubmitted: widget.viewOnSubmitted,
      viewLeading: widget.viewLeading,
      viewTrailing: widget.viewTrailing,
      viewHintText: widget.viewHintText,
      viewBackgroundColor: widget.viewBackgroundColor,
      viewElevation: widget.viewElevation,
      viewSurfaceTintColor: widget.viewSurfaceTintColor,
      viewSide: widget.viewSide,
      viewShape: widget.viewShape,
      viewBarPadding: widget.viewBarPadding,
      viewHeaderHeight: widget.headerHeight,
      viewHeaderTextStyle: widget.headerTextStyle,
      viewHeaderHintStyle: widget.headerHintStyle,
      dividerColor: widget.dividerColor,
      viewConstraints: widget.viewConstraints,
      viewPadding: widget.viewPadding,
      shrinkWrap: widget.shrinkWrap,
      showFullScreenView: getShowFullScreenView(),
      toggleVisibility: toggleVisibility,
      textDirection: Directionality.of(context),
      viewBuilder: widget.viewBuilder,
      anchorKey: _anchorKey,
      searchController: _searchController,
      suggestionsBuilder: widget.suggestionsBuilder,
      textCapitalization: widget.textCapitalization,
      capturedThemes: InheritedTheme.capture(from: context, to: navigator.context),
      textInputAction: widget.textInputAction,
      keyboardType: widget.keyboardType,
    );
    navigator.push(_route!);
  }

  void _closeView(String? selectedText) {
    if (selectedText != null) {
      _searchController.text = selectedText;
    }
    Navigator.of(context).pop();
  }

  bool toggleVisibility() {
    setState(() {
      _anchorIsVisible = !_anchorIsVisible;
    });
    return _anchorIsVisible;
  }

  bool getShowFullScreenView() {
    return widget.isFullScreen ??
        switch (Theme.of(context).platform) {
          TargetPlatform.iOS || TargetPlatform.android || TargetPlatform.fuchsia => true,
          TargetPlatform.macOS || TargetPlatform.linux || TargetPlatform.windows => false,
        };
  }

  double _getOpacity() {
    if (widget.enabled) {
      return _anchorIsVisible ? 1.0 : 0.0;
    }
    return _kDisableSearchBarOpacity;
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedOpacity(
      key: _anchorKey,
      opacity: _getOpacity(),
      duration: _kAnchorFadeDuration,
      child: IgnorePointer(
        ignoring: !widget.enabled,
        child: GestureDetector(onTap: _openView, child: widget.builder(context, _searchController)),
      ),
    );
  }
}

class _SearchViewRoute extends PopupRoute<_SearchViewRoute> {
  _SearchViewRoute({
    this.viewOnChanged,
    this.viewOnSubmitted,
    this.toggleVisibility,
    this.textDirection,
    this.viewBuilder,
    this.viewLeading,
    this.viewTrailing,
    this.viewHintText,
    this.viewBackgroundColor,
    this.viewElevation,
    this.viewSurfaceTintColor,
    this.viewSide,
    this.viewShape,
    this.viewBarPadding,
    this.viewHeaderHeight,
    this.viewHeaderTextStyle,
    this.viewHeaderHintStyle,
    this.dividerColor,
    this.viewConstraints,
    this.viewPadding,
    this.shrinkWrap,
    this.textCapitalization,
    required this.showFullScreenView,
    required this.anchorKey,
    required this.searchController,
    required this.suggestionsBuilder,
    required this.capturedThemes,
    this.textInputAction,
    this.keyboardType,
  });

  final ValueChanged<String>? viewOnChanged;
  final ValueChanged<String>? viewOnSubmitted;
  final ValueGetter<bool>? toggleVisibility;
  final TextDirection? textDirection;
  final ViewBuilder? viewBuilder;
  final Widget? viewLeading;
  final Iterable<Widget>? viewTrailing;
  final String? viewHintText;
  final Color? viewBackgroundColor;
  final double? viewElevation;
  final Color? viewSurfaceTintColor;
  final BorderSide? viewSide;
  final OutlinedBorder? viewShape;
  final EdgeInsetsGeometry? viewBarPadding;
  final double? viewHeaderHeight;
  final TextStyle? viewHeaderTextStyle;
  final TextStyle? viewHeaderHintStyle;
  final Color? dividerColor;
  final BoxConstraints? viewConstraints;
  final EdgeInsetsGeometry? viewPadding;
  final bool? shrinkWrap;
  final TextCapitalization? textCapitalization;
  final bool showFullScreenView;
  final GlobalKey anchorKey;
  final SearchController searchController;
  final SuggestionsBuilder suggestionsBuilder;
  final CapturedThemes capturedThemes;
  final TextInputAction? textInputAction;
  final TextInputType? keyboardType;
  CurvedAnimation? curvedAnimation;
  CurvedAnimation? viewFadeOnIntervalCurve;
  bool willDisposeSearchController = false;

  @override
  Color? get barrierColor => Colors.transparent;

  @override
  bool get barrierDismissible => true;

  @override
  String? get barrierLabel => 'Dismiss';

  late final SearchViewThemeData viewDefaults;
  late final SearchViewThemeData viewTheme;
  final RectTween _rectTween = RectTween();

  Rect? getRect() {
    final BuildContext? context = anchorKey.currentContext;
    if (context != null) {
      final RenderBox searchBarBox = context.findRenderObject()! as RenderBox;
      final Size boxSize = searchBarBox.size;
      final NavigatorState navigator = Navigator.of(context);
      final Offset boxLocation = searchBarBox.localToGlobal(
        Offset.zero,
        ancestor: navigator.context.findRenderObject(),
      );
      return boxLocation & boxSize;
    }
    return null;
  }

  @override
  TickerFuture didPush() {
    assert(anchorKey.currentContext != null);
    updateViewConfig(anchorKey.currentContext!);
    updateTweens(anchorKey.currentContext!);
    toggleVisibility?.call();
    return super.didPush();
  }

  @override
  bool didPop(_SearchViewRoute? result) {
    assert(anchorKey.currentContext != null);
    updateTweens(anchorKey.currentContext!);
    toggleVisibility?.call();
    return super.didPop(result);
  }

  void _dismiss({required bool disposeController}) {
    willDisposeSearchController = disposeController;
    if (isActive) {
      navigator?.removeRoute(this);
    }
  }

  @override
  void dispose() {
    curvedAnimation?.dispose();
    viewFadeOnIntervalCurve?.dispose();
    if (willDisposeSearchController) {
      searchController.dispose();
    }
    super.dispose();
  }

  void updateViewConfig(BuildContext context) {
    viewDefaults = _SearchViewDefaultsM3(context, isFullScreen: showFullScreenView);
    viewTheme = SearchViewTheme.of(context);
  }

  void updateTweens(BuildContext context) {
    final RenderBox navigator = Navigator.of(context).context.findRenderObject()! as RenderBox;
    final Size screenSize = navigator.size;
    final Rect anchorRect = getRect() ?? Rect.zero;

    final BoxConstraints effectiveConstraints =
        viewConstraints ?? viewTheme.constraints ?? viewDefaults.constraints!;
    _rectTween.begin = anchorRect;

    final double viewWidth = clampDouble(
      anchorRect.width,
      effectiveConstraints.minWidth,
      effectiveConstraints.maxWidth,
    );
    final double viewHeight = clampDouble(
      screenSize.height * 2 / 3,
      effectiveConstraints.minHeight,
      effectiveConstraints.maxHeight,
    );

    switch (textDirection ?? TextDirection.ltr) {
      case TextDirection.ltr:
        final double viewLeftToScreenRight = screenSize.width - anchorRect.left;
        final double viewTopToScreenBottom = screenSize.height - anchorRect.top;

        // Make sure the search view doesn't go off the screen. If the search view
        // doesn't fit, move the top-left corner of the view to fit the window.
        // If the window is smaller than the view, then we resize the view to fit the window.
        Offset topLeft = anchorRect.topLeft;
        if (viewLeftToScreenRight < viewWidth) {
          topLeft = Offset(screenSize.width - math.min(viewWidth, screenSize.width), topLeft.dy);
        }
        if (viewTopToScreenBottom < viewHeight) {
          topLeft = Offset(topLeft.dx, screenSize.height - math.min(viewHeight, screenSize.height));
        }
        final Size endSize = Size(viewWidth, viewHeight);
        _rectTween.end = showFullScreenView ? Offset.zero & screenSize : (topLeft & endSize);
        return;
      case TextDirection.rtl:
        final double viewRightToScreenLeft = anchorRect.right;
        final double viewTopToScreenBottom = screenSize.height - anchorRect.top;

        // Make sure the search view doesn't go off the screen.
        Offset topLeft = Offset(math.max(anchorRect.right - viewWidth, 0.0), anchorRect.top);
        if (viewRightToScreenLeft < viewWidth) {
          topLeft = Offset(0.0, topLeft.dy);
        }
        if (viewTopToScreenBottom < viewHeight) {
          topLeft = Offset(topLeft.dx, screenSize.height - math.min(viewHeight, screenSize.height));
        }
        final Size endSize = Size(viewWidth, viewHeight);
        _rectTween.end = showFullScreenView ? Offset.zero & screenSize : (topLeft & endSize);
    }
  }

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return Directionality(
      textDirection: textDirection ?? TextDirection.ltr,
      child: AnimatedBuilder(
        animation: animation,
        builder: (BuildContext context, Widget? child) {
          curvedAnimation ??= CurvedAnimation(
            parent: animation,
            curve: Curves.easeInOutCubicEmphasized,
            reverseCurve: Curves.easeInOutCubicEmphasized.flipped,
          );

          final Rect viewRect = _rectTween.evaluate(curvedAnimation!)!;
          final double topPadding =
              showFullScreenView
                  ? lerpDouble(0.0, MediaQuery.paddingOf(context).top, curvedAnimation!.value)!
                  : 0.0;

          viewFadeOnIntervalCurve ??= CurvedAnimation(
            parent: animation,
            curve: _kViewFadeOnInterval,
            reverseCurve: _kViewFadeOnInterval.flipped,
          );

          return FadeTransition(
            opacity: viewFadeOnIntervalCurve!,
            child: capturedThemes.wrap(
              _ViewContent(
                viewOnChanged: viewOnChanged,
                viewOnSubmitted: viewOnSubmitted,
                viewLeading: viewLeading,
                viewTrailing: viewTrailing,
                viewHintText: viewHintText,
                viewBackgroundColor: viewBackgroundColor,
                viewElevation: viewElevation,
                viewSurfaceTintColor: viewSurfaceTintColor,
                viewSide: viewSide,
                viewShape: viewShape,
                viewBarPadding: viewBarPadding,
                viewHeaderHeight: viewHeaderHeight,
                viewHeaderTextStyle: viewHeaderTextStyle,
                viewHeaderHintStyle: viewHeaderHintStyle,
                dividerColor: dividerColor,
                viewConstraints: viewConstraints,
                viewPadding: viewPadding,
                shrinkWrap: shrinkWrap,
                showFullScreenView: showFullScreenView,
                animation: curvedAnimation!,
                topPadding: topPadding,
                viewMaxWidth: _rectTween.end!.width,
                viewRect: viewRect,
                viewBuilder: viewBuilder,
                searchController: searchController,
                suggestionsBuilder: suggestionsBuilder,
                textCapitalization: textCapitalization,
                textInputAction: textInputAction,
                keyboardType: keyboardType,
              ),
            ),
          );
        },
      ),
    );
  }

  @override
  Duration get transitionDuration => _kOpenViewDuration;
}

class _ViewContent extends StatefulWidget {
  const _ViewContent({
    this.viewOnChanged,
    this.viewOnSubmitted,
    this.viewBuilder,
    this.viewLeading,
    this.viewTrailing,
    this.viewHintText,
    this.viewBackgroundColor,
    this.viewElevation,
    this.viewSurfaceTintColor,
    this.viewSide,
    this.viewShape,
    this.viewBarPadding,
    this.viewHeaderHeight,
    this.viewHeaderTextStyle,
    this.viewHeaderHintStyle,
    this.dividerColor,
    this.viewConstraints,
    this.viewPadding,
    this.shrinkWrap,
    this.textCapitalization,
    required this.showFullScreenView,
    required this.topPadding,
    required this.animation,
    required this.viewMaxWidth,
    required this.viewRect,
    required this.searchController,
    required this.suggestionsBuilder,
    this.textInputAction,
    this.keyboardType,
  });

  final ValueChanged<String>? viewOnChanged;
  final ValueChanged<String>? viewOnSubmitted;
  final ViewBuilder? viewBuilder;
  final Widget? viewLeading;
  final Iterable<Widget>? viewTrailing;
  final String? viewHintText;
  final Color? viewBackgroundColor;
  final double? viewElevation;
  final Color? viewSurfaceTintColor;
  final BorderSide? viewSide;
  final OutlinedBorder? viewShape;
  final EdgeInsetsGeometry? viewBarPadding;
  final double? viewHeaderHeight;
  final TextStyle? viewHeaderTextStyle;
  final TextStyle? viewHeaderHintStyle;
  final Color? dividerColor;
  final BoxConstraints? viewConstraints;
  final EdgeInsetsGeometry? viewPadding;
  final bool? shrinkWrap;
  final TextCapitalization? textCapitalization;
  final bool showFullScreenView;
  final double topPadding;
  final Animation<double> animation;
  final double viewMaxWidth;
  final Rect viewRect;
  final SearchController searchController;
  final SuggestionsBuilder suggestionsBuilder;
  final TextInputAction? textInputAction;
  final TextInputType? keyboardType;

  @override
  State<_ViewContent> createState() => _ViewContentState();
}

class _ViewContentState extends State<_ViewContent> {
  Size? _screenSize;
  late Rect _viewRect;
  late CurvedAnimation viewIconsFadeCurve;
  late CurvedAnimation viewDividerFadeCurve;
  late CurvedAnimation viewListFadeOnIntervalCurve;
  late final SearchController _controller;
  Iterable<Widget> result = <Widget>[];
  String? searchValue;
  Timer? _timer;

  @override
  void initState() {
    super.initState();
    _viewRect = widget.viewRect;
    _controller = widget.searchController;
    _controller.addListener(updateSuggestions);
    _setupAnimations();
  }

  @override
  void didUpdateWidget(covariant _ViewContent oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.viewRect != oldWidget.viewRect) {
      setState(() {
        _viewRect = widget.viewRect;
      });
    }
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final Size updatedScreenSize = MediaQuery.of(context).size;

    if (_screenSize != updatedScreenSize) {
      _screenSize = updatedScreenSize;
      if (widget.showFullScreenView) {
        _viewRect = Offset.zero & _screenSize!;
      }
    }

    if (searchValue != _controller.text) {
      _timer?.cancel();
      _timer = Timer(Duration.zero, () async {
        searchValue = _controller.text;
        final Iterable<Widget> suggestions = await widget.suggestionsBuilder(context, _controller);
        _timer?.cancel();
        _timer = null;
        if (mounted) {
          setState(() {
            result = suggestions;
          });
        }
      });
    }
  }

  @override
  void dispose() {
    _controller.removeListener(updateSuggestions);
    _disposeAnimations();
    _timer?.cancel();
    _timer = null;
    super.dispose();
  }

  void _setupAnimations() {
    viewIconsFadeCurve = CurvedAnimation(
      parent: widget.animation,
      curve: _kViewIconsFadeOnInterval,
      reverseCurve: _kViewIconsFadeOnInterval.flipped,
    );
    viewDividerFadeCurve = CurvedAnimation(
      parent: widget.animation,
      curve: _kViewDividerFadeOnInterval,
      reverseCurve: _kViewFadeOnInterval.flipped,
    );
    viewListFadeOnIntervalCurve = CurvedAnimation(
      parent: widget.animation,
      curve: _kViewListFadeOnInterval,
      reverseCurve: _kViewListFadeOnInterval.flipped,
    );
  }

  void _disposeAnimations() {
    viewIconsFadeCurve.dispose();
    viewDividerFadeCurve.dispose();
    viewListFadeOnIntervalCurve.dispose();
  }

  Future<void> updateSuggestions() async {
    if (searchValue != _controller.text) {
      searchValue = _controller.text;
      final Iterable<Widget> suggestions = await widget.suggestionsBuilder(context, _controller);
      if (mounted) {
        setState(() {
          result = suggestions;
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final Widget defaultLeading = BackButton(
      style: const ButtonStyle(tapTargetSize: MaterialTapTargetSize.shrinkWrap),
      onPressed: () {
        Navigator.of(context).pop();
      },
    );

    final List<Widget> defaultTrailing = <Widget>[
      if (_controller.text.isNotEmpty)
        IconButton(
          icon: const Icon(Icons.close),
          tooltip: MaterialLocalizations.of(context).clearButtonTooltip,
          onPressed: () {
            _controller.clear();
          },
        ),
    ];

    final SearchViewThemeData viewDefaults = _SearchViewDefaultsM3(
      context,
      isFullScreen: widget.showFullScreenView,
    );
    final SearchViewThemeData viewTheme = SearchViewTheme.of(context);
    final DividerThemeData dividerTheme = DividerTheme.of(context);

    final Color effectiveBackgroundColor =
        widget.viewBackgroundColor ?? viewTheme.backgroundColor ?? viewDefaults.backgroundColor!;
    final Color effectiveSurfaceTint =
        widget.viewSurfaceTintColor ?? viewTheme.surfaceTintColor ?? viewDefaults.surfaceTintColor!;
    final double effectiveElevation =
        widget.viewElevation ?? viewTheme.elevation ?? viewDefaults.elevation!;
    final BorderSide? effectiveSide = widget.viewSide ?? viewTheme.side ?? viewDefaults.side;
    OutlinedBorder effectiveShape = widget.viewShape ?? viewTheme.shape ?? viewDefaults.shape!;
    if (effectiveSide != null) {
      effectiveShape = effectiveShape.copyWith(side: effectiveSide);
    }
    final Color effectiveDividerColor =
        widget.dividerColor ??
        viewTheme.dividerColor ??
        dividerTheme.color ??
        viewDefaults.dividerColor!;
    final double? effectiveHeaderHeight = widget.viewHeaderHeight ?? viewTheme.headerHeight;
    final BoxConstraints? headerConstraints =
        effectiveHeaderHeight == null
            ? null
            : BoxConstraints.tightFor(height: effectiveHeaderHeight);
    final TextStyle? effectiveTextStyle =
        widget.viewHeaderTextStyle ?? viewTheme.headerTextStyle ?? viewDefaults.headerTextStyle;
    final TextStyle? effectiveHintStyle =
        widget.viewHeaderHintStyle ??
        viewTheme.headerHintStyle ??
        widget.viewHeaderTextStyle ??
        viewTheme.headerTextStyle ??
        viewDefaults.headerHintStyle;
    final EdgeInsetsGeometry? effectivePadding =
        widget.viewPadding ?? viewTheme.padding ?? viewDefaults.padding;
    final EdgeInsetsGeometry? effectiveBarPadding =
        widget.viewBarPadding ?? viewTheme.barPadding ?? viewDefaults.barPadding;

    final BoxConstraints effectiveConstraints =
        widget.viewConstraints ?? viewTheme.constraints ?? viewDefaults.constraints!;
    final double minWidth = math.min(effectiveConstraints.minWidth, _viewRect.width);
    final double minHeight = math.min(effectiveConstraints.minHeight, _viewRect.height);

    final bool effectiveShrinkWrap =
        widget.shrinkWrap ?? viewTheme.shrinkWrap ?? viewDefaults.shrinkWrap!;

    final Widget viewDivider = DividerTheme(
      data: dividerTheme.copyWith(color: effectiveDividerColor),
      child: const Divider(height: 1),
    );

    return Align(
      alignment: Alignment.topLeft,
      child: Transform.translate(
        offset: _viewRect.topLeft,
        child: ConstrainedBox(
          constraints: BoxConstraints(
            minWidth: minWidth,
            maxWidth: _viewRect.width,
            minHeight: minHeight,
            maxHeight: _viewRect.height,
          ),
          child: Padding(
            padding:
                widget.showFullScreenView ? EdgeInsets.zero : (effectivePadding ?? EdgeInsets.zero),
            child: Material(
              clipBehavior: Clip.antiAlias,
              shape: effectiveShape,
              color: effectiveBackgroundColor,
              surfaceTintColor: effectiveSurfaceTint,
              elevation: effectiveElevation,
              child: OverflowBox(
                alignment: Alignment.topLeft,
                maxWidth: math.min(widget.viewMaxWidth, _screenSize!.width),
                minWidth: 0,
                fit: OverflowBoxFit.deferToChild,
                child: FadeTransition(
                  opacity: viewIconsFadeCurve,
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: <Widget>[
                      Padding(
                        padding: EdgeInsets.only(top: widget.topPadding),
                        child: SafeArea(
                          top: false,
                          bottom: false,
                          child: SearchBar(
                            autoFocus: true,
                            constraints:
                                headerConstraints ??
                                (widget.showFullScreenView
                                    ? BoxConstraints(
                                      minHeight: _SearchViewDefaultsM3.fullScreenBarHeight,
                                    )
                                    : null),
                            padding: WidgetStatePropertyAll<EdgeInsetsGeometry?>(
                              effectiveBarPadding,
                            ),
                            leading: widget.viewLeading ?? defaultLeading,
                            trailing: widget.viewTrailing ?? defaultTrailing,
                            hintText: widget.viewHintText,
                            backgroundColor: const MaterialStatePropertyAll<Color>(
                              Colors.transparent,
                            ),
                            overlayColor: const MaterialStatePropertyAll<Color>(Colors.transparent),
                            elevation: const MaterialStatePropertyAll<double>(0.0),
                            textStyle: MaterialStatePropertyAll<TextStyle?>(effectiveTextStyle),
                            hintStyle: MaterialStatePropertyAll<TextStyle?>(effectiveHintStyle),
                            controller: _controller,
                            onChanged: (String value) {
                              widget.viewOnChanged?.call(value);
                              updateSuggestions();
                            },
                            onSubmitted: widget.viewOnSubmitted,
                            textCapitalization: widget.textCapitalization,
                            textInputAction: widget.textInputAction,
                            keyboardType: widget.keyboardType,
                          ),
                        ),
                      ),
                      if (!effectiveShrinkWrap ||
                          minHeight > 0 ||
                          widget.showFullScreenView ||
                          result.isNotEmpty) ...<Widget>[
                        FadeTransition(opacity: viewDividerFadeCurve, child: viewDivider),
                        Flexible(
                          fit:
                              (effectiveShrinkWrap && !widget.showFullScreenView)
                                  ? FlexFit.loose
                                  : FlexFit.tight,
                          child: FadeTransition(
                            opacity: viewListFadeOnIntervalCurve,
                            child:
                                widget.viewBuilder == null
                                    ? MediaQuery.removePadding(
                                      context: context,
                                      removeTop: true,
                                      child: ListView(
                                        shrinkWrap: effectiveShrinkWrap,
                                        children: result.toList(),
                                      ),
                                    )
                                    : widget.viewBuilder!(result),
                          ),
                        ),
                      ],
                    ],
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

class _SearchAnchorWithSearchBar extends SearchAnchor {
  _SearchAnchorWithSearchBar({
    Widget? barLeading,
    Iterable<Widget>? barTrailing,
    String? barHintText,
    GestureTapCallback? onTap,
    MaterialStateProperty<double?>? barElevation,
    MaterialStateProperty<Color?>? barBackgroundColor,
    MaterialStateProperty<Color?>? barOverlayColor,
    MaterialStateProperty<BorderSide?>? barSide,
    MaterialStateProperty<OutlinedBorder?>? barShape,
    MaterialStateProperty<EdgeInsetsGeometry?>? barPadding,
    super.viewBarPadding,
    MaterialStateProperty<TextStyle?>? barTextStyle,
    MaterialStateProperty<TextStyle?>? barHintStyle,
    super.viewBuilder,
    super.viewLeading,
    super.viewTrailing,
    String? viewHintText,
    super.viewBackgroundColor,
    super.viewElevation,
    super.viewSide,
    super.viewShape,
    double? viewHeaderHeight,
    TextStyle? viewHeaderTextStyle,
    TextStyle? viewHeaderHintStyle,
    super.dividerColor,
    BoxConstraints? constraints,
    super.viewConstraints,
    super.viewPadding,
    super.shrinkWrap,
    super.isFullScreen,
    super.searchController,
    super.textCapitalization,
    ValueChanged<String>? onChanged,
    ValueChanged<String>? onSubmitted,
    required super.suggestionsBuilder,
    super.textInputAction,
    super.keyboardType,
    EdgeInsets scrollPadding = const EdgeInsets.all(20.0),
    EditableTextContextMenuBuilder contextMenuBuilder = SearchBar._defaultContextMenuBuilder,
  }) : super(
         viewHintText: viewHintText ?? barHintText,
         headerHeight: viewHeaderHeight,
         headerTextStyle: viewHeaderTextStyle,
         headerHintStyle: viewHeaderHintStyle,
         viewOnSubmitted: onSubmitted,
         viewOnChanged: onChanged,
         builder: (BuildContext context, SearchController controller) {
           return SearchBar(
             constraints: constraints,
             controller: controller,
             onTap: () {
               controller.openView();
               onTap?.call();
             },
             onChanged: (String value) {
               controller.openView();
             },
             onSubmitted: onSubmitted,
             hintText: barHintText,
             hintStyle: barHintStyle,
             textStyle: barTextStyle,
             elevation: barElevation,
             backgroundColor: barBackgroundColor,
             overlayColor: barOverlayColor,
             side: barSide,
             shape: barShape,
             padding:
                 barPadding ??
                 const MaterialStatePropertyAll<EdgeInsets>(EdgeInsets.symmetric(horizontal: 16.0)),
             leading: barLeading ?? const Icon(Icons.search),
             trailing: barTrailing,
             textCapitalization: textCapitalization,
             textInputAction: textInputAction,
             keyboardType: keyboardType,
             scrollPadding: scrollPadding,
             contextMenuBuilder: contextMenuBuilder,
           );
         },
       );
}

/// A controller to manage a search view created by [SearchAnchor].
///
/// A [SearchController] is used to control a menu after it has been created,
/// with methods such as [openView] and [closeView]. It can also control the text in the
/// input field.
///
/// See also:
///
/// * [SearchAnchor], a widget that defines a region that opens a search view.
/// * [TextEditingController], A controller for an editable text field.
class SearchController extends TextEditingController {
  // The anchor that this controller controls.
  //
  // This is set automatically when a [SearchController] is given to the anchor
  // it controls.
  _SearchAnchorState? _anchor;

  /// Whether this controller has associated search anchor.
  bool get isAttached => _anchor != null;

  /// Whether or not the associated search view is currently open.
  bool get isOpen {
    assert(isAttached);
    return _anchor!._viewIsOpen;
  }

  /// Opens the search view that this controller is associated with.
  void openView() {
    assert(isAttached);
    _anchor!._openView();
  }

  /// Close the search view that this search controller is associated with.
  ///
  /// If `selectedText` is given, then the text value of the controller is set to
  /// `selectedText`.
  void closeView(String? selectedText) {
    assert(isAttached);
    _anchor!._closeView(selectedText);
  }

  // ignore: use_setters_to_change_properties
  void _attach(_SearchAnchorState anchor) {
    _anchor = anchor;
  }

  void _detach(_SearchAnchorState anchor) {
    if (_anchor == anchor) {
      _anchor = null;
    }
  }
}

/// A Material Design search bar.
///
/// A [SearchBar] looks like a [TextField]. Tapping a SearchBar typically shows a
/// "search view" route: a route with the search bar at the top and a list of
/// suggested completions for the search bar's text below. [SearchBar]s are
/// usually created by a [SearchAnchor.builder]. The builder provides a
/// [SearchController] that's used by the search bar's [SearchBar.onTap] or
/// [SearchBar.onChanged] callbacks to show the search view and to hide it
/// when the user selects a suggestion.
///
/// For [TextDirection.ltr], the [leading] widget is on the left side of the bar.
/// It should contain either a navigational action (such as a menu or up-arrow)
/// or a non-functional search icon.
///
/// The [trailing] is an optional list that appears at the other end of
/// the search bar. Typically only one or two action icons are included.
/// These actions can represent additional modes of searching (like voice search),
/// a separate high-level action (such as current location) or an overflow menu.
///
/// {@tool dartpad}
/// This example demonstrates how to use a [SearchBar] as the return value of the
/// [SearchAnchor.builder] property. The [SearchBar] also includes a leading search
/// icon and a trailing action to toggle the brightness.
///
/// ** See code in examples/api/lib/material/search_anchor/search_bar.0.dart **
/// {@end-tool}
///
/// See also:
///
/// * [SearchAnchor], a widget that typically uses an [IconButton] or a [SearchBar]
/// to manage a "search view" route.
/// * [SearchBarTheme], a widget that overrides the default configuration of a search bar.
/// * [SearchViewTheme], a widget that overrides the default configuration of a search view.
class SearchBar extends StatefulWidget {
  /// Creates a Material Design search bar.
  const SearchBar({
    super.key,
    this.controller,
    this.focusNode,
    this.hintText,
    this.leading,
    this.trailing,
    this.onTap,
    this.onTapOutside,
    this.onChanged,
    this.onSubmitted,
    this.constraints,
    this.elevation,
    this.backgroundColor,
    this.shadowColor,
    this.surfaceTintColor,
    this.overlayColor,
    this.side,
    this.shape,
    this.padding,
    this.textStyle,
    this.hintStyle,
    this.textCapitalization,
    this.enabled = true,
    this.autoFocus = false,
    this.textInputAction,
    this.keyboardType,
    this.scrollPadding = const EdgeInsets.all(20.0),
    this.contextMenuBuilder = _defaultContextMenuBuilder,
  });

  /// Controls the text being edited in the search bar's text field.
  ///
  /// If null, this widget will create its own [TextEditingController].
  final TextEditingController? controller;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// Text that suggests what sort of input the field accepts.
  ///
  /// Displayed at the same location on the screen where text may be entered
  /// when the input is empty.
  ///
  /// Defaults to null.
  final String? hintText;

  /// A widget to display before the text input field.
  ///
  /// Typically the [leading] widget is an [Icon] or an [IconButton].
  final Widget? leading;

  /// A list of Widgets to display in a row after the text field.
  ///
  /// Typically these actions can represent additional modes of searching
  /// (like voice search), an avatar, a separate high-level action (such as
  /// current location) or an overflow menu. There should not be more than
  /// two trailing actions.
  final Iterable<Widget>? trailing;

  /// Called when the user taps this search bar.
  final GestureTapCallback? onTap;

  /// Called when the user taps outside the search bar.
  final TapRegionCallback? onTapOutside;

  /// Invoked upon user input.
  final ValueChanged<String>? onChanged;

  /// Called when the user indicates that they are done editing the text in the
  /// field.
  final ValueChanged<String>? onSubmitted;

  /// Optional size constraints for the search bar.
  ///
  /// If null, the value of [SearchBarThemeData.constraints] will be used. If
  /// this is also null, then the constraints defaults to:
  /// ```dart
  /// const BoxConstraints(minWidth: 360.0, maxWidth: 800.0, minHeight: 56.0)
  /// ```
  final BoxConstraints? constraints;

  /// The elevation of the search bar's [Material].
  ///
  /// If null, the value of [SearchBarThemeData.elevation] will be used. If this
  /// is also null, then default value is 6.0.
  final MaterialStateProperty<double?>? elevation;

  /// The search bar's background fill color.
  ///
  /// If null, the value of [SearchBarThemeData.backgroundColor] will be used.
  /// If this is also null, then the default value is [ColorScheme.surfaceContainerHigh].
  final MaterialStateProperty<Color?>? backgroundColor;

  /// The shadow color of the search bar's [Material].
  ///
  /// If null, the value of [SearchBarThemeData.shadowColor] will be used.
  /// If this is also null, then the default value is [ColorScheme.shadow].
  final MaterialStateProperty<Color?>? shadowColor;

  /// The surface tint color of the search bar's [Material].
  ///
  /// This is not recommended for use. [Material 3 spec](https://m3.material.io/styles/color/the-color-system/color-roles)
  /// introduced a set of tone-based surfaces and surface containers in its [ColorScheme],
  /// which provide more flexibility. The intention is to eventually remove surface tint color from
  /// the framework.
  ///
  /// If null, the value of [SearchBarThemeData.surfaceTintColor] will be used.
  /// If this is also null, then the default value is [Colors.transparent].
  final MaterialStateProperty<Color?>? surfaceTintColor;

  /// The highlight color that's typically used to indicate that
  /// the search bar is focused, hovered, or pressed.
  final MaterialStateProperty<Color?>? overlayColor;

  /// The color and weight of the search bar's outline.
  ///
  /// This value is combined with [shape] to create a shape decorated
  /// with an outline.
  ///
  /// If null, the value of [SearchBarThemeData.side] will be used. If this is
  /// also null, the search bar doesn't have a side by default.
  final MaterialStateProperty<BorderSide?>? side;

  /// The shape of the search bar's underlying [Material].
  ///
  /// This shape is combined with [side] to create a shape decorated
  /// with an outline.
  ///
  /// If null, the value of [SearchBarThemeData.shape] will be used.
  /// If this is also null, defaults to [StadiumBorder].
  final MaterialStateProperty<OutlinedBorder?>? shape;

  /// The padding between the search bar's boundary and its contents.
  ///
  /// If null, the value of [SearchBarThemeData.padding] will be used.
  /// If this is also null, then the default value is 16.0 horizontally.
  final MaterialStateProperty<EdgeInsetsGeometry?>? padding;

  /// The style to use for the text being edited.
  ///
  /// If null, defaults to the `bodyLarge` text style from the current [Theme].
  /// The default text color is [ColorScheme.onSurface].
  final MaterialStateProperty<TextStyle?>? textStyle;

  /// The style to use for the [hintText].
  ///
  /// If null, the value of [SearchBarThemeData.hintStyle] will be used. If this
  /// is also null, the value of [textStyle] will be used. If this is also null,
  /// defaults to the `bodyLarge` text style from the current [Theme].
  /// The default text color is [ColorScheme.onSurfaceVariant].
  final MaterialStateProperty<TextStyle?>? hintStyle;

  /// {@macro flutter.widgets.editableText.textCapitalization}
  final TextCapitalization? textCapitalization;

  /// Whether or not this widget is currently interactive.
  ///
  /// When false, the widget will ignore taps and appear dimmed.
  ///
  /// Defaults to true.
  final bool enabled;

  /// {@macro flutter.widgets.editableText.autofocus}
  final bool autoFocus;

  /// {@macro flutter.widgets.TextField.textInputAction}
  final TextInputAction? textInputAction;

  /// The type of action button to use for the keyboard.
  ///
  /// Defaults to the default value specified in [TextField].
  final TextInputType? keyboardType;

  /// {@macro flutter.widgets.editableText.scrollPadding}
  final EdgeInsets scrollPadding;

  /// {@macro flutter.widgets.EditableText.contextMenuBuilder}
  ///
  /// If not provided, will build a default menu based on the platform.
  ///
  /// See also:
  ///
  ///  * [AdaptiveTextSelectionToolbar], which is built by default.
  ///  * [BrowserContextMenu], which allows the browser's context menu on web to
  ///    be disabled and Flutter-rendered context menus to appear.
  final EditableTextContextMenuBuilder? contextMenuBuilder;

  static Widget _defaultContextMenuBuilder(
    BuildContext context,
    EditableTextState editableTextState,
  ) {
    return AdaptiveTextSelectionToolbar.editableText(editableTextState: editableTextState);
  }

  @override
  State<SearchBar> createState() => _SearchBarState();
}

class _SearchBarState extends State<SearchBar> {
  late final MaterialStatesController _internalStatesController;
  FocusNode? _internalFocusNode;
  FocusNode get _focusNode => widget.focusNode ?? (_internalFocusNode ??= FocusNode());

  @override
  void initState() {
    super.initState();
    _internalStatesController = MaterialStatesController();
    _internalStatesController.addListener(() {
      setState(() {});
    });
  }

  @override
  void dispose() {
    _internalStatesController.dispose();
    _internalFocusNode?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final TextDirection textDirection = Directionality.of(context);
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final SearchBarThemeData searchBarTheme = SearchBarTheme.of(context);
    final SearchBarThemeData defaults = _SearchBarDefaultsM3(context);

    T? resolve<T>(
      MaterialStateProperty<T>? widgetValue,
      MaterialStateProperty<T>? themeValue,
      MaterialStateProperty<T>? defaultValue,
    ) {
      final Set<MaterialState> states = _internalStatesController.value;
      return widgetValue?.resolve(states) ??
          themeValue?.resolve(states) ??
          defaultValue?.resolve(states);
    }

    final TextStyle? effectiveTextStyle = resolve<TextStyle?>(
      widget.textStyle,
      searchBarTheme.textStyle,
      defaults.textStyle,
    );
    final double? effectiveElevation = resolve<double?>(
      widget.elevation,
      searchBarTheme.elevation,
      defaults.elevation,
    );
    final Color? effectiveShadowColor = resolve<Color?>(
      widget.shadowColor,
      searchBarTheme.shadowColor,
      defaults.shadowColor,
    );
    final Color? effectiveBackgroundColor = resolve<Color?>(
      widget.backgroundColor,
      searchBarTheme.backgroundColor,
      defaults.backgroundColor,
    );
    final Color? effectiveSurfaceTintColor = resolve<Color?>(
      widget.surfaceTintColor,
      searchBarTheme.surfaceTintColor,
      defaults.surfaceTintColor,
    );
    final OutlinedBorder? effectiveShape = resolve<OutlinedBorder?>(
      widget.shape,
      searchBarTheme.shape,
      defaults.shape,
    );
    final BorderSide? effectiveSide = resolve<BorderSide?>(
      widget.side,
      searchBarTheme.side,
      defaults.side,
    );
    final EdgeInsetsGeometry? effectivePadding = resolve<EdgeInsetsGeometry?>(
      widget.padding,
      searchBarTheme.padding,
      defaults.padding,
    );
    final MaterialStateProperty<Color?>? effectiveOverlayColor =
        widget.overlayColor ?? searchBarTheme.overlayColor ?? defaults.overlayColor;
    final TextCapitalization effectiveTextCapitalization =
        widget.textCapitalization ??
        searchBarTheme.textCapitalization ??
        defaults.textCapitalization!;

    final Set<MaterialState> states = _internalStatesController.value;
    final TextStyle? effectiveHintStyle =
        widget.hintStyle?.resolve(states) ??
        searchBarTheme.hintStyle?.resolve(states) ??
        widget.textStyle?.resolve(states) ??
        searchBarTheme.textStyle?.resolve(states) ??
        defaults.hintStyle?.resolve(states);

    final Color defaultColor = switch (colorScheme.brightness) {
      Brightness.light => kDefaultIconDarkColor,
      Brightness.dark => kDefaultIconLightColor,
    };
    final IconThemeData? customTheme = switch (IconTheme.of(context)) {
      final IconThemeData iconTheme when iconTheme.color != defaultColor => iconTheme,
      _ => null,
    };

    Widget? leading;
    if (widget.leading != null) {
      leading = IconTheme.merge(
        data: customTheme ?? IconThemeData(color: colorScheme.onSurface),
        child: widget.leading!,
      );
    }

    final List<Widget>? trailing =
        widget.trailing
            ?.map(
              (Widget trailing) => IconTheme.merge(
                data: customTheme ?? IconThemeData(color: colorScheme.onSurfaceVariant),
                child: trailing,
              ),
            )
            .toList();

    return ConstrainedBox(
      constraints: widget.constraints ?? searchBarTheme.constraints ?? defaults.constraints!,
      child: Opacity(
        opacity: widget.enabled ? 1 : _kDisableSearchBarOpacity,
        child: Material(
          elevation: effectiveElevation!,
          shadowColor: effectiveShadowColor,
          color: effectiveBackgroundColor,
          surfaceTintColor: effectiveSurfaceTintColor,
          shape: effectiveShape?.copyWith(side: effectiveSide),
          child: IgnorePointer(
            ignoring: !widget.enabled,
            child: InkWell(
              onTap: () {
                widget.onTap?.call();
                if (!_focusNode.hasFocus) {
                  _focusNode.requestFocus();
                }
              },
              overlayColor: effectiveOverlayColor,
              customBorder: effectiveShape?.copyWith(side: effectiveSide),
              statesController: _internalStatesController,
              child: Padding(
                padding: effectivePadding!,
                child: Row(
                  textDirection: textDirection,
                  children: <Widget>[
                    if (leading != null) leading,
                    Expanded(
                      child: Padding(
                        padding: effectivePadding,
                        child: TextField(
                          autofocus: widget.autoFocus,
                          onTap: widget.onTap,
                          onTapAlwaysCalled: true,
                          onTapOutside: widget.onTapOutside,
                          focusNode: _focusNode,
                          onChanged: widget.onChanged,
                          onSubmitted: widget.onSubmitted,
                          controller: widget.controller,
                          style: effectiveTextStyle,
                          enabled: widget.enabled,
                          decoration: InputDecoration(hintText: widget.hintText).applyDefaults(
                            InputDecorationTheme(
                              hintStyle: effectiveHintStyle,
                              // The configuration below is to make sure that the text field
                              // in `SearchBar` will not be overridden by the overall `InputDecorationTheme`
                              enabledBorder: InputBorder.none,
                              border: InputBorder.none,
                              focusedBorder: InputBorder.none,
                              contentPadding: EdgeInsets.zero,
                              // Setting `isDense` to true to allow the text field height to be
                              // smaller than 48.0
                              isDense: true,
                            ),
                          ),
                          textCapitalization: effectiveTextCapitalization,
                          textInputAction: widget.textInputAction,
                          keyboardType: widget.keyboardType,
                          scrollPadding: widget.scrollPadding,
                          contextMenuBuilder: widget.contextMenuBuilder,
                        ),
                      ),
                    ),
                    ...?trailing,
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

// BEGIN GENERATED TOKEN PROPERTIES - SearchBar

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _SearchBarDefaultsM3 extends SearchBarThemeData {
  _SearchBarDefaultsM3(this.context);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  MaterialStateProperty<Color?>? get backgroundColor =>
    MaterialStatePropertyAll<Color>(_colors.surfaceContainerHigh);

  @override
  MaterialStateProperty<double>? get elevation =>
    const MaterialStatePropertyAll<double>(6.0);

  @override
  MaterialStateProperty<Color>? get shadowColor =>
    MaterialStatePropertyAll<Color>(_colors.shadow);

  @override
  MaterialStateProperty<Color>? get surfaceTintColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<Color?>? get overlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return _colors.onSurface.withOpacity(0.1);
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onSurface.withOpacity(0.08);
      }
      if (states.contains(MaterialState.focused)) {
        return Colors.transparent;
      }
      return Colors.transparent;
    });

  // No default side

  @override
  MaterialStateProperty<OutlinedBorder>? get shape =>
    const MaterialStatePropertyAll<OutlinedBorder>(StadiumBorder());

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding =>
    const MaterialStatePropertyAll<EdgeInsetsGeometry>(EdgeInsets.symmetric(horizontal: 8.0));

  @override
  MaterialStateProperty<TextStyle?> get textStyle =>
    MaterialStatePropertyAll<TextStyle?>(_textTheme.bodyLarge?.copyWith(color: _colors.onSurface));

  @override
  MaterialStateProperty<TextStyle?> get hintStyle =>
    MaterialStatePropertyAll<TextStyle?>(_textTheme.bodyLarge?.copyWith(color: _colors.onSurfaceVariant));

  @override
  BoxConstraints get constraints =>
    const BoxConstraints(minWidth: 360.0, maxWidth: 800.0, minHeight: 56.0);

  @override
  TextCapitalization get textCapitalization => TextCapitalization.none;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - SearchBar

// BEGIN GENERATED TOKEN PROPERTIES - SearchView

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _SearchViewDefaultsM3 extends SearchViewThemeData {
  _SearchViewDefaultsM3(this.context, {required this.isFullScreen});

  final BuildContext context;
  final bool isFullScreen;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  static double fullScreenBarHeight = 72.0;

  @override
  Color? get backgroundColor => _colors.surfaceContainerHigh;

  @override
  double? get elevation => 6.0;

  @override
  Color? get surfaceTintColor => Colors.transparent;

  // No default side

  @override
  OutlinedBorder? get shape => isFullScreen
    ? const RoundedRectangleBorder()
    : const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(28.0)));

  @override
  TextStyle? get headerTextStyle => _textTheme.bodyLarge?.copyWith(color: _colors.onSurface);

  @override
  TextStyle? get headerHintStyle => _textTheme.bodyLarge?.copyWith(color: _colors.onSurfaceVariant);

  @override
  BoxConstraints get constraints => const BoxConstraints(minWidth: 360.0, minHeight: 240.0);

  @override
  EdgeInsetsGeometry? get barPadding => const EdgeInsets.symmetric(horizontal: 8.0);

  @override
  bool get shrinkWrap => false;

  @override
  Color? get dividerColor => _colors.outline;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - SearchView
