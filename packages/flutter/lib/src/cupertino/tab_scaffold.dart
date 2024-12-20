// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/services.dart';
///
/// @docImport 'page_scaffold.dart';
/// @docImport 'route.dart';
/// @docImport 'tab_view.dart';
library;

import 'package:flutter/widgets.dart';

import 'bottom_tab_bar.dart';
import 'colors.dart';
import 'theme.dart';

/// Coordinates tab selection between a [CupertinoTabBar] and a [CupertinoTabScaffold].
///
/// The [index] property is the index of the selected tab. Changing its value
/// updates the actively displayed tab of the [CupertinoTabScaffold] the
/// [CupertinoTabController] controls, as well as the currently selected tab item of
/// its [CupertinoTabBar].
///
/// {@tool dartpad}
/// This samples shows how [CupertinoTabController] can be used to switch tabs in
/// [CupertinoTabScaffold].
///
/// ** See code in examples/api/lib/cupertino/tab_scaffold/cupertino_tab_controller.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoTabScaffold], a tabbed application root layout that can be
///    controlled by a [CupertinoTabController].
///  * [RestorableCupertinoTabController], which is a restorable version
///    of this controller.
class CupertinoTabController extends ChangeNotifier {
  /// Creates a [CupertinoTabController] to control the tab index of [CupertinoTabScaffold]
  /// and [CupertinoTabBar].
  ///
  /// The [initialIndex] defaults to 0. The value must be greater than or equal
  /// to 0, and less than the total number of tabs.
  CupertinoTabController({int initialIndex = 0}) : _index = initialIndex, assert(initialIndex >= 0);

  bool _isDisposed = false;

  /// The index of the currently selected tab.
  ///
  /// Changing the value of [index] updates the actively displayed tab of the
  /// [CupertinoTabScaffold] controlled by this [CupertinoTabController], as well
  /// as the currently selected tab item of its [CupertinoTabScaffold.tabBar].
  ///
  /// The value must be greater than or equal to 0, and less than the total
  /// number of tabs.
  int get index => _index;
  int _index;
  set index(int value) {
    assert(value >= 0);
    if (_index == value) {
      return;
    }
    _index = value;
    notifyListeners();
  }

  @mustCallSuper
  @override
  void dispose() {
    super.dispose();
    _isDisposed = true;
  }
}

/// Implements a tabbed iOS application's root layout and behavior structure.
///
/// The scaffold lays out the tab bar at the bottom and the content between or
/// behind the tab bar.
///
/// A [tabBar] and a [tabBuilder] are required. The [CupertinoTabScaffold]
/// will automatically listen to the provided [CupertinoTabBar]'s tap callbacks
/// to change the active tab.
///
/// A [controller] can be used to provide an initially selected tab index and manage
/// subsequent tab changes. If a controller is not specified, the scaffold will
/// create its own [CupertinoTabController] and manage it internally. Otherwise
/// it's up to the owner of [controller] to call `dispose` on it after finish
/// using it.
///
/// Tabs' contents are built with the provided [tabBuilder] at the active
/// tab index. The [tabBuilder] must be able to build the same number of
/// pages as there are [tabBar] items. Inactive tabs will be moved [Offstage]
/// and their animations disabled.
///
/// Adding/removing tabs, or changing the order of tabs is supported but not
/// recommended. Doing so is against the iOS human interface guidelines, and
/// [CupertinoTabScaffold] may lose some tabs' state in the process.
///
/// Use [CupertinoTabView] as the root widget of each tab to support tabs with
/// parallel navigation state and history. Since each [CupertinoTabView] contains
/// a [Navigator], rebuilding the [CupertinoTabView] with a different
/// [WidgetBuilder] instance in [CupertinoTabView.builder] will not recreate
/// the [CupertinoTabView]'s navigation stack or update its UI. To update the
/// contents of the [CupertinoTabView] after it's built, trigger a rebuild
/// (via [State.setState], for instance) from its descendant rather than from
/// its ancestor.
///
/// {@tool dartpad}
/// A sample code implementing a typical iOS information architecture with tabs.
///
/// ** See code in examples/api/lib/cupertino/tab_scaffold/cupertino_tab_scaffold.0.dart **
/// {@end-tool}
///
/// To push a route above all tabs instead of inside the currently selected one
/// (such as when showing a dialog on top of this scaffold), use
/// `Navigator.of(rootNavigator: true)` from inside the [BuildContext] of a
/// [CupertinoTabView].
///
/// See also:
///
///  * [CupertinoTabBar], the bottom tab bar inserted in the scaffold.
///  * [CupertinoTabController], the selection state of this widget.
///  * [CupertinoTabView], the typical root content of each tab that holds its own
///    [Navigator] stack.
///  * [CupertinoPageRoute], a route hosting modal pages with iOS style transitions.
///  * [CupertinoPageScaffold], typical contents of an iOS modal page implementing
///    layout with a navigation bar on top.
///  * [iOS human interface guidelines](https://developer.apple.com/design/human-interface-guidelines/ios/bars/tab-bars/).
class CupertinoTabScaffold extends StatefulWidget {
  /// Creates a layout for applications with a tab bar at the bottom.
  CupertinoTabScaffold({
    super.key,
    required this.tabBar,
    required this.tabBuilder,
    this.controller,
    this.backgroundColor,
    this.resizeToAvoidBottomInset = true,
    this.restorationId,
  }) : assert(
         controller == null || controller.index < tabBar.items.length,
         "The CupertinoTabController's current index ${controller.index} is "
         'out of bounds for the tab bar with ${tabBar.items.length} tabs',
       );

  /// The [tabBar] is a [CupertinoTabBar] drawn at the bottom of the screen
  /// that lets the user switch between different tabs in the main content area
  /// when present.
  ///
  /// The [CupertinoTabBar.currentIndex] is only used to initialize a
  /// [CupertinoTabController] when no [controller] is provided. Subsequently
  /// providing a different [CupertinoTabBar.currentIndex] does not affect the
  /// scaffold or the tab bar's active tab index. To programmatically change
  /// the active tab index, use a [CupertinoTabController].
  ///
  /// If [CupertinoTabBar.onTap] is provided, it will still be called.
  /// [CupertinoTabScaffold] automatically also listen to the
  /// [CupertinoTabBar]'s `onTap` to change the [controller]'s `index`
  /// and change the actively displayed tab in [CupertinoTabScaffold]'s own
  /// main content area.
  ///
  /// If translucent, the main content may slide behind it.
  /// Otherwise, the main content's bottom margin will be offset by its height.
  ///
  /// By default [tabBar] disables text scaling to match the native iOS behavior.
  /// To override this behavior, wrap each of the [tabBar]'s items inside a
  /// [MediaQuery] with the desired [TextScaler].
  final CupertinoTabBar tabBar;

  /// Controls the currently selected tab index of the [tabBar], as well as the
  /// active tab index of the [tabBuilder]. Providing a different [controller]
  /// will also update the scaffold's current active index to the new controller's
  /// index value.
  ///
  /// Defaults to null.
  final CupertinoTabController? controller;

  /// An [IndexedWidgetBuilder] that's called when tabs become active.
  ///
  /// The widgets built by [IndexedWidgetBuilder] are typically a
  /// [CupertinoTabView] in order to achieve the parallel hierarchical
  /// information architecture seen on iOS apps with tab bars.
  ///
  /// When the tab becomes inactive, its content is cached in the widget tree
  /// [Offstage] and its animations disabled.
  ///
  /// Content can slide under the [tabBar] when they're translucent.
  /// In that case, the child's [BuildContext]'s [MediaQuery] will have a
  /// bottom padding indicating the area of obstructing overlap from the
  /// [tabBar].
  final IndexedWidgetBuilder tabBuilder;

  /// The color of the widget that underlies the entire scaffold.
  ///
  /// By default uses [CupertinoTheme]'s `scaffoldBackgroundColor` when null.
  final Color? backgroundColor;

  /// Whether the body should size itself to avoid the window's bottom inset.
  ///
  /// For example, if there is an onscreen keyboard displayed above the
  /// scaffold, the body can be resized to avoid overlapping the keyboard, which
  /// prevents widgets inside the body from being obscured by the keyboard.
  ///
  /// Defaults to true.
  final bool resizeToAvoidBottomInset;

  /// Restoration ID to save and restore the state of the [CupertinoTabScaffold].
  ///
  /// This property only has an effect when no [controller] has been provided:
  /// If it is non-null (and no [controller] has been provided), the scaffold
  /// will persist and restore the currently selected tab index. If a
  /// [controller] has been provided, it is the responsibility of the owner of
  /// that controller to persist and restore it, e.g. by using a
  /// [RestorableCupertinoTabController].
  ///
  /// The state of this widget is persisted in a [RestorationBucket] claimed
  /// from the surrounding [RestorationScope] using the provided restoration ID.
  ///
  /// See also:
  ///
  ///  * [RestorationManager], which explains how state restoration works in
  ///    Flutter.
  final String? restorationId;

  @override
  State<CupertinoTabScaffold> createState() => _CupertinoTabScaffoldState();
}

class _CupertinoTabScaffoldState extends State<CupertinoTabScaffold> with RestorationMixin {
  RestorableCupertinoTabController? _internalController;
  CupertinoTabController get _controller => widget.controller ?? _internalController!.value;

  @override
  String? get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    _restoreInternalController();
  }

  void _restoreInternalController() {
    if (_internalController != null) {
      registerForRestoration(_internalController!, 'controller');
      _internalController!.value.addListener(_onCurrentIndexChange);
    }
  }

  @override
  void initState() {
    super.initState();
    _updateTabController();
  }

  void _updateTabController([CupertinoTabController? oldWidgetController]) {
    if (widget.controller == null && _internalController == null) {
      // No widget-provided controller: create an internal controller.
      _internalController = RestorableCupertinoTabController(
        initialIndex: widget.tabBar.currentIndex,
      );
      if (!restorePending) {
        _restoreInternalController(); // Also adds the listener to the controller.
      }
    }
    if (widget.controller != null && _internalController != null) {
      // Use the widget-provided controller.
      unregisterFromRestoration(_internalController!);
      _internalController!.dispose();
      _internalController = null;
    }
    if (oldWidgetController != widget.controller) {
      // The widget-provided controller has changed: move listeners.
      if (oldWidgetController?._isDisposed == false) {
        oldWidgetController!.removeListener(_onCurrentIndexChange);
      }
      widget.controller?.addListener(_onCurrentIndexChange);
    }
  }

  void _onCurrentIndexChange() {
    assert(
      _controller.index >= 0 && _controller.index < widget.tabBar.items.length,
      "The $runtimeType's current index ${_controller.index} is "
      'out of bounds for the tab bar with ${widget.tabBar.items.length} tabs',
    );

    // The value of `_controller.index` has already been updated at this point.
    // Calling `setState` to rebuild using `_controller.index`.
    setState(() {});
  }

  @override
  void didUpdateWidget(CupertinoTabScaffold oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.controller != oldWidget.controller) {
      _updateTabController(oldWidget.controller);
    } else if (_controller.index >= widget.tabBar.items.length) {
      // If a new [tabBar] with less than (_controller.index + 1) items is provided,
      // clamp the current index.
      _controller.index = widget.tabBar.items.length - 1;
    }
  }

  @override
  Widget build(BuildContext context) {
    final MediaQueryData existingMediaQuery = MediaQuery.of(context);
    MediaQueryData newMediaQuery = MediaQuery.of(context);

    Widget content = _TabSwitchingView(
      currentTabIndex: _controller.index,
      tabCount: widget.tabBar.items.length,
      tabBuilder: widget.tabBuilder,
    );
    EdgeInsets contentPadding = EdgeInsets.zero;

    if (widget.resizeToAvoidBottomInset) {
      // Remove the view inset and add it back as a padding in the inner content.
      newMediaQuery = newMediaQuery.removeViewInsets(removeBottom: true);
      contentPadding = EdgeInsets.only(bottom: existingMediaQuery.viewInsets.bottom);
    }

    // Only pad the content with the height of the tab bar if the tab
    // isn't already entirely obstructed by a keyboard or other view insets.
    // Don't double pad.
    if (!widget.resizeToAvoidBottomInset ||
        widget.tabBar.preferredSize.height > existingMediaQuery.viewInsets.bottom) {
      // TODO(xster): Use real size after partial layout instead of preferred size.
      // https://github.com/flutter/flutter/issues/12912
      final double bottomPadding =
          widget.tabBar.preferredSize.height + existingMediaQuery.padding.bottom;

      // If tab bar opaque, directly stop the main content higher. If
      // translucent, let main content draw behind the tab bar but hint the
      // obstructed area.
      if (widget.tabBar.opaque(context)) {
        contentPadding = EdgeInsets.only(bottom: bottomPadding);
        newMediaQuery = newMediaQuery.removePadding(removeBottom: true);
      } else {
        newMediaQuery = newMediaQuery.copyWith(
          padding: newMediaQuery.padding.copyWith(bottom: bottomPadding),
        );
      }
    }

    content = MediaQuery(
      data: newMediaQuery,
      child: Padding(padding: contentPadding, child: content),
    );

    return DecoratedBox(
      decoration: BoxDecoration(
        color:
            CupertinoDynamicColor.maybeResolve(widget.backgroundColor, context) ??
            CupertinoTheme.of(context).scaffoldBackgroundColor,
      ),
      child: Stack(
        children: <Widget>[
          // The main content being at the bottom is added to the stack first.
          content,
          MediaQuery.withNoTextScaling(
            child: Align(
              alignment: Alignment.bottomCenter,
              // Override the tab bar's currentIndex to the current tab and hook in
              // our own listener to update the [_controller.currentIndex] on top of a possibly user
              // provided callback.
              child: widget.tabBar.copyWith(
                currentIndex: _controller.index,
                onTap: (int newIndex) {
                  _controller.index = newIndex;
                  // Chain the user's original callback.
                  widget.tabBar.onTap?.call(newIndex);
                },
              ),
            ),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    if (widget.controller?._isDisposed == false) {
      _controller.removeListener(_onCurrentIndexChange);
    }
    _internalController?.dispose();
    super.dispose();
  }
}

/// A widget laying out multiple tabs with only one active tab being built
/// at a time and on stage. Off stage tabs' animations are stopped.
class _TabSwitchingView extends StatefulWidget {
  const _TabSwitchingView({
    required this.currentTabIndex,
    required this.tabCount,
    required this.tabBuilder,
  }) : assert(tabCount > 0);

  final int currentTabIndex;
  final int tabCount;
  final IndexedWidgetBuilder tabBuilder;

  @override
  _TabSwitchingViewState createState() => _TabSwitchingViewState();
}

class _TabSwitchingViewState extends State<_TabSwitchingView> {
  final List<bool> shouldBuildTab = <bool>[];
  final List<FocusScopeNode> tabFocusNodes = <FocusScopeNode>[];

  // When focus nodes are no longer needed, we need to dispose of them, but we
  // can't be sure that nothing else is listening to them until this widget is
  // disposed of, so when they are no longer needed, we move them to this list,
  // and dispose of them when we dispose of this widget.
  final List<FocusScopeNode> discardedNodes = <FocusScopeNode>[];

  @override
  void initState() {
    super.initState();
    shouldBuildTab.addAll(List<bool>.filled(widget.tabCount, false));
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _focusActiveTab();
  }

  @override
  void didUpdateWidget(_TabSwitchingView oldWidget) {
    super.didUpdateWidget(oldWidget);

    // Only partially invalidate the tabs cache to avoid breaking the current
    // behavior. We assume that the only possible change is either:
    // - new tabs are appended to the tab list, or
    // - some trailing tabs are removed.
    // If the above assumption is not true, some tabs may lose their state.
    final int lengthDiff = widget.tabCount - shouldBuildTab.length;
    if (lengthDiff > 0) {
      shouldBuildTab.addAll(List<bool>.filled(lengthDiff, false));
    } else if (lengthDiff < 0) {
      shouldBuildTab.removeRange(widget.tabCount, shouldBuildTab.length);
    }
    _focusActiveTab();
  }

  // Will focus the active tab if the FocusScope above it has focus already. If
  // not, then it will just mark it as the preferred focus for that scope.
  void _focusActiveTab() {
    if (tabFocusNodes.length != widget.tabCount) {
      if (tabFocusNodes.length > widget.tabCount) {
        discardedNodes.addAll(tabFocusNodes.sublist(widget.tabCount));
        tabFocusNodes.removeRange(widget.tabCount, tabFocusNodes.length);
      } else {
        tabFocusNodes.addAll(
          List<FocusScopeNode>.generate(
            widget.tabCount - tabFocusNodes.length,
            (int index) => FocusScopeNode(
              debugLabel: '$CupertinoTabScaffold Tab ${index + tabFocusNodes.length}',
            ),
          ),
        );
      }
    }
    FocusScope.of(context).setFirstFocus(tabFocusNodes[widget.currentTabIndex]);
  }

  @override
  void dispose() {
    for (final FocusScopeNode focusScopeNode in tabFocusNodes) {
      focusScopeNode.dispose();
    }
    for (final FocusScopeNode focusScopeNode in discardedNodes) {
      focusScopeNode.dispose();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      fit: StackFit.expand,
      children: List<Widget>.generate(widget.tabCount, (int index) {
        final bool active = index == widget.currentTabIndex;
        shouldBuildTab[index] = active || shouldBuildTab[index];

        return HeroMode(
          enabled: active,
          child: Offstage(
            offstage: !active,
            child: TickerMode(
              enabled: active,
              child: FocusScope(
                node: tabFocusNodes[index],
                child: Builder(
                  builder: (BuildContext context) {
                    return shouldBuildTab[index]
                        ? widget.tabBuilder(context, index)
                        : const SizedBox.shrink();
                  },
                ),
              ),
            ),
          ),
        );
      }),
    );
  }
}

/// A [RestorableProperty] that knows how to store and restore a
/// [CupertinoTabController].
///
/// The [CupertinoTabController] is accessible via the [value] getter. During
/// state restoration, the property will restore [CupertinoTabController.index]
/// to the value it had when the restoration data it is getting restored from
/// was collected.
class RestorableCupertinoTabController extends RestorableChangeNotifier<CupertinoTabController> {
  /// Creates a [RestorableCupertinoTabController] to control the tab index of
  /// [CupertinoTabScaffold] and [CupertinoTabBar].
  ///
  /// The `initialIndex` defaults to zero. The value must be greater than or
  /// equal to zero, and less than the total number of tabs.
  RestorableCupertinoTabController({int initialIndex = 0})
    : assert(initialIndex >= 0),
      _initialIndex = initialIndex;

  final int _initialIndex;

  @override
  CupertinoTabController createDefaultValue() {
    return CupertinoTabController(initialIndex: _initialIndex);
  }

  @override
  CupertinoTabController fromPrimitives(Object? data) {
    assert(data != null);
    return CupertinoTabController(initialIndex: data! as int);
  }

  @override
  Object? toPrimitives() {
    return value.index;
  }
}
