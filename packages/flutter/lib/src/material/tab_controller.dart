// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app_bar.dart';
/// @docImport 'scaffold.dart';
/// @docImport 'tabs.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'constants.dart';

// Examples can assume:
// late BuildContext context;

/// Coordinates tab selection between a [TabBar] and a [TabBarView].
///
/// The [index] property is the index of the selected tab and the [animation]
/// represents the current scroll positions of the tab bar and the tab bar view.
/// The selected tab's index can be changed with [animateTo].
///
/// A stateful widget that builds a [TabBar] or a [TabBarView] can create
/// a [TabController] and share it directly.
///
/// When the [TabBar] and [TabBarView] don't have a convenient stateful
/// ancestor, a [TabController] can be shared by providing a
/// [DefaultTabController] inherited widget.
///
/// {@animation 700 540 https://flutter.github.io/assets-for-api-docs/assets/material/tabs.mp4}
///
/// {@tool snippet}
///
/// This widget introduces a [Scaffold] with an [AppBar] and a [TabBar].
///
/// ```dart
/// class MyTabbedPage extends StatefulWidget {
///   const MyTabbedPage({ super.key });
///   @override
///   State<MyTabbedPage> createState() => _MyTabbedPageState();
/// }
///
/// class _MyTabbedPageState extends State<MyTabbedPage> with SingleTickerProviderStateMixin {
///   static const List<Tab> myTabs = <Tab>[
///     Tab(text: 'LEFT'),
///     Tab(text: 'RIGHT'),
///   ];
///
///   late TabController _tabController;
///
///   @override
///   void initState() {
///     super.initState();
///     _tabController = TabController(vsync: this, length: myTabs.length);
///   }
///
///  @override
///  void dispose() {
///    _tabController.dispose();
///    super.dispose();
///  }
///
///   @override
///   Widget build(BuildContext context) {
///     return Scaffold(
///       appBar: AppBar(
///         bottom: TabBar(
///           controller: _tabController,
///           tabs: myTabs,
///         ),
///       ),
///       body: TabBarView(
///         controller: _tabController,
///         children: myTabs.map((Tab tab) {
///           final String label = tab.text!.toLowerCase();
///           return Center(
///             child: Text(
///               'This is the $label tab',
///               style: const TextStyle(fontSize: 36),
///             ),
///           );
///         }).toList(),
///       ),
///     );
///   }
/// }
/// ```
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to listen to page updates in [TabBar] and [TabBarView]
/// when using [DefaultTabController].
///
/// ** See code in examples/api/lib/material/tab_controller/tab_controller.1.dart **
/// {@end-tool}
///
class TabController extends ChangeNotifier {
  /// Creates an object that manages the state required by [TabBar] and a
  /// [TabBarView].
  ///
  /// The [length] must not be negative. Typically it's a value greater than
  /// one, i.e. typically there are two or more tabs. The [length] must match
  /// [TabBar.tabs]'s and [TabBarView.children]'s length.
  ///
  /// The `initialIndex` must be valid given [length]. If [length] is zero, then
  /// `initialIndex` must be 0 (the default).
  TabController({
    int initialIndex = 0,
    Duration? animationDuration,
    required this.length,
    required TickerProvider vsync,
  }) : assert(length >= 0),
       assert(initialIndex >= 0 && (length == 0 || initialIndex < length)),
       _index = initialIndex,
       _previousIndex = initialIndex,
       _animationDuration = animationDuration ?? kTabScrollDuration,
       _animationController = AnimationController.unbounded(
         value: initialIndex.toDouble(),
         vsync: vsync,
       ) {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  // Private constructor used by `_copyWith`. This allows a new TabController to
  // be created without having to create a new animationController.
  TabController._({
    required int index,
    required int previousIndex,
    required AnimationController? animationController,
    required Duration animationDuration,
    required this.length,
  }) : _index = index,
       _previousIndex = previousIndex,
       _animationController = animationController,
       _animationDuration = animationDuration {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  /// Creates a new [TabController] with `index`, `previousIndex`, `length`, and
  /// `animationDuration` if they are non-null, and disposes current instance.
  ///
  /// This method is used by [DefaultTabController].
  ///
  /// When [DefaultTabController.length] is updated, this method is called to
  /// create a new [TabController] without creating a new [AnimationController].
  /// Instead the [_animationController] is nulled in current instance and
  /// passed to the new instance.
  TabController _copyWithAndDispose({
    required int? index,
    required int? length,
    required int? previousIndex,
    required Duration? animationDuration,
  }) {
    if (index != null) {
      _animationController!.value = index.toDouble();
    }
    final TabController result = TabController._(
      index: index ?? _index,
      length: length ?? this.length,
      animationController: _animationController,
      previousIndex: previousIndex ?? _previousIndex,
      animationDuration: animationDuration ?? _animationDuration,
    );
    _animationController = null;
    dispose();
    return result;
  }

  /// An animation whose value represents the current position of the [TabBar]'s
  /// selected tab indicator as well as the scrollOffsets of the [TabBar]
  /// and [TabBarView].
  ///
  /// The animation's value ranges from 0.0 to [length] - 1.0. After the
  /// selected tab is changed, the animation's value equals [index]. The
  /// animation's value can be [offset] by +/- 1.0 to reflect [TabBarView]
  /// drag scrolling.
  ///
  /// If this [TabController] was disposed, then return null.
  Animation<double>? get animation => _animationController?.view;
  AnimationController? _animationController;

  /// Controls the duration of TabController and TabBarView animations.
  ///
  /// Defaults to kTabScrollDuration.
  Duration get animationDuration => _animationDuration;
  final Duration _animationDuration;

  /// The total number of tabs.
  ///
  /// Typically greater than one. Must match [TabBar.tabs]'s and
  /// [TabBarView.children]'s length.
  final int length;

  void _changeIndex(int value, {Duration? duration, Curve? curve}) {
    assert(value >= 0 && (value < length || length == 0));
    assert(duration != null || curve == null);
    assert(_indexIsChangingCount >= 0);
    if (value == _index || length < 2) {
      return;
    }
    _previousIndex = index;
    _index = value;
    if (duration != null && duration > Duration.zero) {
      _indexIsChangingCount += 1;
      notifyListeners(); // Because the value of indexIsChanging may have changed.
      _animationController!
          .animateTo(_index.toDouble(), duration: duration, curve: curve!)
          .whenCompleteOrCancel(() {
            if (_animationController != null) {
              // don't notify if we've been disposed
              _indexIsChangingCount -= 1;
              notifyListeners();
            }
          });
    } else {
      _indexIsChangingCount += 1;
      _animationController!.value = _index.toDouble();
      _indexIsChangingCount -= 1;
      notifyListeners();
    }
  }

  /// The index of the currently selected tab.
  ///
  /// Changing the index also updates [previousIndex], sets the [animation]'s
  /// value to index, resets [indexIsChanging] to false, and notifies listeners.
  ///
  /// To change the currently selected tab and play the [animation] use [animateTo].
  ///
  /// The value of [index] must be valid given [length]. If [length] is zero,
  /// then [index] will also be zero.
  int get index => _index;
  int _index;
  set index(int value) {
    _changeIndex(value);
  }

  /// The index of the previously selected tab.
  ///
  /// Initially the same as [index].
  int get previousIndex => _previousIndex;
  int _previousIndex;

  /// True while we're animating from [previousIndex] to [index] as a
  /// consequence of calling [animateTo].
  ///
  /// This value is true during the [animateTo] animation that's triggered when
  /// the user taps a [TabBar] tab. It is false when [offset] is changing as a
  /// consequence of the user dragging (and "flinging") the [TabBarView].
  bool get indexIsChanging => _indexIsChangingCount != 0;
  int _indexIsChangingCount = 0;

  /// Immediately sets [index] and [previousIndex] and then plays the
  /// [animation] from its current value to [index].
  ///
  /// While the animation is running [indexIsChanging] is true. When the
  /// animation completes [offset] will be 0.0.
  void animateTo(int value, {Duration? duration, Curve curve = Curves.ease}) {
    _changeIndex(value, duration: duration ?? _animationDuration, curve: curve);
  }

  /// The difference between the [animation]'s value and [index].
  ///
  /// The offset value must be between -1.0 and 1.0.
  ///
  /// This property is typically set by the [TabBarView] when the user
  /// drags left or right. A value between -1.0 and 0.0 implies that the
  /// TabBarView has been dragged to the left. Similarly a value between
  /// 0.0 and 1.0 implies that the TabBarView has been dragged to the right.
  double get offset => _animationController!.value - _index.toDouble();
  set offset(double value) {
    assert(value >= -1.0 && value <= 1.0);
    assert(!indexIsChanging);
    if (value == offset) {
      return;
    }
    _animationController!.value = value + _index.toDouble();
  }

  @override
  void dispose() {
    _animationController?.dispose();
    _animationController = null;
    super.dispose();
  }
}

class _TabControllerScope extends InheritedWidget {
  const _TabControllerScope({
    required this.controller,
    required this.enabled,
    required super.child,
  });

  final TabController controller;
  final bool enabled;

  @override
  bool updateShouldNotify(_TabControllerScope old) {
    return enabled != old.enabled || controller != old.controller;
  }
}

/// The [TabController] for descendant widgets that don't specify one
/// explicitly.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=POtoEH-5l40}
///
/// [DefaultTabController] is an inherited widget that is used to share a
/// [TabController] with a [TabBar] or a [TabBarView]. It's used when sharing an
/// explicitly created [TabController] isn't convenient because the tab bar
/// widgets are created by a stateless parent widget or by different parent
/// widgets.
///
/// {@animation 700 540 https://flutter.github.io/assets-for-api-docs/assets/material/tabs.mp4}
///
/// ```dart
/// class MyDemo extends StatelessWidget {
///   const MyDemo({super.key});
///
///   static const List<Tab> myTabs = <Tab>[
///     Tab(text: 'LEFT'),
///     Tab(text: 'RIGHT'),
///   ];
///
///   @override
///   Widget build(BuildContext context) {
///     return DefaultTabController(
///       length: myTabs.length,
///       child: Scaffold(
///         appBar: AppBar(
///           bottom: const TabBar(
///             tabs: myTabs,
///           ),
///         ),
///         body: TabBarView(
///           children: myTabs.map((Tab tab) {
///             final String label = tab.text!.toLowerCase();
///             return Center(
///               child: Text(
///                 'This is the $label tab',
///                 style: const TextStyle(fontSize: 36),
///               ),
///             );
///           }).toList(),
///         ),
///       ),
///     );
///   }
/// }
/// ```
class DefaultTabController extends StatefulWidget {
  /// Creates a default tab controller for the given [child] widget.
  ///
  /// The [length] argument is typically greater than one. The [length] must
  /// match [TabBar.tabs]'s and [TabBarView.children]'s length.
  const DefaultTabController({
    super.key,
    required this.length,
    this.initialIndex = 0,
    required this.child,
    this.animationDuration,
  }) : assert(length >= 0),
       assert(length == 0 || (initialIndex >= 0 && initialIndex < length));

  /// The total number of tabs.
  ///
  /// Typically greater than one. Must match [TabBar.tabs]'s and
  /// [TabBarView.children]'s length.
  final int length;

  /// The initial index of the selected tab.
  ///
  /// Defaults to zero.
  final int initialIndex;

  /// Controls the duration of DefaultTabController and TabBarView animations.
  ///
  /// Defaults to kTabScrollDuration.
  final Duration? animationDuration;

  /// The widget below this widget in the tree.
  ///
  /// Typically a [Scaffold] whose [AppBar] includes a [TabBar].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The closest instance of [DefaultTabController] that encloses the given
  /// context, or null if none is found.
  ///
  /// {@tool snippet} Typical usage is as follows:
  ///
  /// ```dart
  /// TabController? controller = DefaultTabController.maybeOf(context);
  /// ```
  /// {@end-tool}
  ///
  /// Calling this method will create a dependency on the closest
  /// [DefaultTabController] in the [context], if there is one.
  ///
  /// See also:
  ///
  /// * [DefaultTabController.of], which is similar to this method, but asserts
  ///   if no [DefaultTabController] ancestor is found.
  static TabController? maybeOf(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<_TabControllerScope>()?.controller;
  }

  /// The closest instance of [DefaultTabController] that encloses the given
  /// context.
  ///
  /// If no instance is found, this method will assert in debug mode and throw
  /// an exception in release mode.
  ///
  /// Calling this method will create a dependency on the closest
  /// [DefaultTabController] in the [context].
  ///
  /// {@tool snippet} Typical usage is as follows:
  ///
  /// ```dart
  /// TabController controller = DefaultTabController.of(context);
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  /// * [DefaultTabController.maybeOf], which is similar to this method, but
  ///   returns null if no [DefaultTabController] ancestor is found.
  static TabController of(BuildContext context) {
    final TabController? controller = maybeOf(context);
    assert(() {
      if (controller == null) {
        throw FlutterError(
          'DefaultTabController.of() was called with a context that does not '
          'contain a DefaultTabController widget.\n'
          'No DefaultTabController widget ancestor could be found starting from '
          'the context that was passed to DefaultTabController.of(). This can '
          'happen because you are using a widget that looks for a DefaultTabController '
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
  State<DefaultTabController> createState() => _DefaultTabControllerState();
}

class _DefaultTabControllerState extends State<DefaultTabController>
    with SingleTickerProviderStateMixin {
  late TabController _controller;

  @override
  void initState() {
    super.initState();
    _controller = TabController(
      vsync: this,
      length: widget.length,
      initialIndex: widget.initialIndex,
      animationDuration: widget.animationDuration,
    );
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return _TabControllerScope(
      controller: _controller,
      enabled: TickerMode.of(context),
      child: widget.child,
    );
  }

  @override
  void didUpdateWidget(DefaultTabController oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.length != widget.length) {
      // If the length is shortened while the last tab is selected, we should
      // automatically update the index of the controller to be the new last tab.
      int? newIndex;
      int previousIndex = _controller.previousIndex;
      if (_controller.index >= widget.length) {
        newIndex = math.max(0, widget.length - 1);
        previousIndex = _controller.index;
      }
      _controller = _controller._copyWithAndDispose(
        length: widget.length,
        animationDuration: widget.animationDuration,
        index: newIndex,
        previousIndex: previousIndex,
      );
    }

    if (oldWidget.animationDuration != widget.animationDuration) {
      _controller = _controller._copyWithAndDispose(
        length: widget.length,
        animationDuration: widget.animationDuration,
        index: _controller.index,
        previousIndex: _controller.previousIndex,
      );
    }
  }
}
