// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'animated_cross_fade.dart';
/// @docImport 'implicit_animations.dart';
library;

import 'package:flutter/foundation.dart';

import 'basic.dart';
import 'framework.dart';
import 'ticker_provider.dart';
import 'transitions.dart';

// Internal representation of a child that, now or in the past, was set on the
// AnimatedSwitcher.child field, but is now in the process of
// transitioning. The internal representation includes fields that we don't want
// to expose to the public API (like the controller).
class _ChildEntry {
  _ChildEntry({
    required this.controller,
    required this.animation,
    required this.transition,
    required this.widgetChild,
  });

  // The animation controller for the child's transition.
  final AnimationController controller;

  // The (curved) animation being used to drive the transition.
  final CurvedAnimation animation;

  // The currently built transition for this child.
  Widget transition;

  // The widget's child at the time this entry was created or updated.
  // Used to rebuild the transition if necessary.
  Widget widgetChild;

  @override
  String toString() => 'Entry#${shortHash(this)}($widgetChild)';
}

/// Signature for builders used to generate custom transitions for
/// [AnimatedSwitcher].
///
/// The `child` should be transitioning in when the `animation` is running in
/// the forward direction.
///
/// The function should return a widget which wraps the given `child`. It may
/// also use the `animation` to inform its transition. It must not return null.
typedef AnimatedSwitcherTransitionBuilder =
    Widget Function(Widget child, Animation<double> animation);

/// Signature for builders used to generate custom layouts for
/// [AnimatedSwitcher].
///
/// The builder should return a widget which contains the given children, laid
/// out as desired. It must not return null. The builder should be able to
/// handle an empty list of `previousChildren`, or a null `currentChild`.
///
/// The `previousChildren` list is an unmodifiable list, sorted with the oldest
/// at the beginning and the newest at the end. It does not include the
/// `currentChild`.
typedef AnimatedSwitcherLayoutBuilder =
    Widget Function(Widget? currentChild, List<Widget> previousChildren);

/// A widget that by default does a cross-fade between a new widget and the
/// widget previously set on the [AnimatedSwitcher] as a child.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=2W7POjFb88g}
///
/// If they are swapped fast enough (i.e. before [duration] elapses), more than
/// one previous child can exist and be transitioning out while the newest one
/// is transitioning in.
///
/// If the "new" child is the same widget type and key as the "old" child, but
/// with different parameters, then [AnimatedSwitcher] will *not* do a
/// transition between them, since as far as the framework is concerned, they
/// are the same widget and the existing widget can be updated with the new
/// parameters. To force the transition to occur, set a [Key] on each child
/// widget that you wish to be considered unique (typically a [ValueKey] on the
/// widget data that distinguishes this child from the others).
///
/// The same key can be used for a new child as was used for an already-outgoing
/// child; the two will not be considered related. (For example, if a progress
/// indicator with key A is first shown, then an image with key B, then another
/// progress indicator with key A again, all in rapid succession, then the old
/// progress indicator and the image will be fading out while a new progress
/// indicator is fading in.)
///
/// The type of transition can be changed from a cross-fade to a custom
/// transition by setting the [transitionBuilder].
///
/// {@tool dartpad}
/// This sample shows a counter that animates the scale of a text widget
/// whenever the value changes.
///
/// ** See code in examples/api/lib/widgets/animated_switcher/animated_switcher.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedCrossFade], which only fades between two children, but also
///    interpolates their sizes, and is reversible.
///  * [AnimatedOpacity], which can be used to switch between nothingness and
///    a given child by fading the child in and out.
///  * [FadeTransition], which [AnimatedSwitcher] uses to perform the transition.
class AnimatedSwitcher extends StatefulWidget {
  /// Creates an [AnimatedSwitcher].
  const AnimatedSwitcher({
    super.key,
    this.child,
    required this.duration,
    this.reverseDuration,
    this.switchInCurve = Curves.linear,
    this.switchOutCurve = Curves.linear,
    this.transitionBuilder = AnimatedSwitcher.defaultTransitionBuilder,
    this.layoutBuilder = AnimatedSwitcher.defaultLayoutBuilder,
  });

  /// The current child widget to display. If there was a previous child, then
  /// that child will be faded out using the [switchOutCurve], while the new
  /// child is faded in with the [switchInCurve], over the [duration].
  ///
  /// If there was no previous child, then this child will fade in using the
  /// [switchInCurve] over the [duration].
  ///
  /// The child is considered to be "new" if it has a different type or [Key]
  /// (see [Widget.canUpdate]).
  ///
  /// To change the kind of transition used, see [transitionBuilder].
  final Widget? child;

  /// The duration of the transition from the old [child] value to the new one.
  ///
  /// This duration is applied to the given [child] when that property is set to
  /// a new child. The same duration is used when fading out, unless
  /// [reverseDuration] is set. Changing [duration] will not affect the
  /// durations of transitions already in progress.
  final Duration duration;

  /// The duration of the transition from the new [child] value to the old one.
  ///
  /// This duration is applied to the given [child] when that property is set to
  /// a new child. Changing [reverseDuration] will not affect the durations of
  /// transitions already in progress.
  ///
  /// If not set, then the value of [duration] is used by default.
  final Duration? reverseDuration;

  /// The animation curve to use when transitioning in a new [child].
  ///
  /// This curve is applied to the given [child] when that property is set to a
  /// new child. Changing [switchInCurve] will not affect the curve of a
  /// transition already in progress.
  ///
  /// The [switchOutCurve] is used when fading out, except that if [child] is
  /// changed while the current child is in the middle of fading in,
  /// [switchInCurve] will be run in reverse from that point instead of jumping
  /// to the corresponding point on [switchOutCurve].
  final Curve switchInCurve;

  /// The animation curve to use when transitioning a previous [child] out.
  ///
  /// This curve is applied to the [child] when the child is faded in (or when
  /// the widget is created, for the first child). Changing [switchOutCurve]
  /// will not affect the curves of already-visible widgets, it only affects the
  /// curves of future children.
  ///
  /// If [child] is changed while the current child is in the middle of fading
  /// in, [switchInCurve] will be run in reverse from that point instead of
  /// jumping to the corresponding point on [switchOutCurve].
  final Curve switchOutCurve;

  /// A function that wraps a new [child] with an animation that transitions
  /// the [child] in when the animation runs in the forward direction and out
  /// when the animation runs in the reverse direction. This is only called
  /// when a new [child] is set (not for each build), or when a new
  /// [transitionBuilder] is set. If a new [transitionBuilder] is set, then
  /// the transition is rebuilt for the current child and all previous children
  /// using the new [transitionBuilder]. The function must not return null.
  ///
  /// The default is [AnimatedSwitcher.defaultTransitionBuilder].
  ///
  /// The animation provided to the builder has the [duration] and
  /// [switchInCurve] or [switchOutCurve] applied as provided when the
  /// corresponding [child] was first provided.
  ///
  /// See also:
  ///
  ///  * [AnimatedSwitcherTransitionBuilder] for more information about
  ///    how a transition builder should function.
  final AnimatedSwitcherTransitionBuilder transitionBuilder;

  /// A function that wraps all of the children that are transitioning out, and
  /// the [child] that's transitioning in, with a widget that lays all of them
  /// out. This is called every time this widget is built. The function must not
  /// return null.
  ///
  /// The default is [AnimatedSwitcher.defaultLayoutBuilder].
  ///
  /// See also:
  ///
  ///  * [AnimatedSwitcherLayoutBuilder] for more information about
  ///    how a layout builder should function.
  final AnimatedSwitcherLayoutBuilder layoutBuilder;

  @override
  State<AnimatedSwitcher> createState() => _AnimatedSwitcherState();

  /// The transition builder used as the default value of [transitionBuilder].
  ///
  /// The new child is given a [FadeTransition] which increases opacity as
  /// the animation goes from 0.0 to 1.0, and decreases when the animation is
  /// reversed.
  ///
  /// This is an [AnimatedSwitcherTransitionBuilder] function.
  static Widget defaultTransitionBuilder(Widget child, Animation<double> animation) {
    return FadeTransition(key: ValueKey<Key?>(child.key), opacity: animation, child: child);
  }

  /// The layout builder used as the default value of [layoutBuilder].
  ///
  /// The new child is placed in a [Stack] that sizes itself to match the
  /// largest of the child or a previous child. The children are centered on
  /// each other.
  ///
  /// This is an [AnimatedSwitcherLayoutBuilder] function.
  static Widget defaultLayoutBuilder(Widget? currentChild, List<Widget> previousChildren) {
    return Stack(
      alignment: Alignment.center,
      children: <Widget>[...previousChildren, if (currentChild != null) currentChild],
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('duration', duration.inMilliseconds, unit: 'ms'));
    properties.add(
      IntProperty(
        'reverseDuration',
        reverseDuration?.inMilliseconds,
        unit: 'ms',
        defaultValue: null,
      ),
    );
  }
}

class _AnimatedSwitcherState extends State<AnimatedSwitcher> with TickerProviderStateMixin {
  _ChildEntry? _currentEntry;
  final Set<_ChildEntry> _outgoingEntries = <_ChildEntry>{};
  List<Widget>? _outgoingWidgets = const <Widget>[];
  int _childNumber = 0;

  @override
  void initState() {
    super.initState();
    _addEntryForNewChild(animate: false);
  }

  @override
  void didUpdateWidget(AnimatedSwitcher oldWidget) {
    super.didUpdateWidget(oldWidget);

    // If the transition builder changed, then update all of the previous
    // transitions.
    if (widget.transitionBuilder != oldWidget.transitionBuilder) {
      _outgoingEntries.forEach(_updateTransitionForEntry);
      if (_currentEntry != null) {
        _updateTransitionForEntry(_currentEntry!);
      }
      _markChildWidgetCacheAsDirty();
    }

    final bool hasNewChild = widget.child != null;
    final bool hasOldChild = _currentEntry != null;
    if (hasNewChild != hasOldChild ||
        hasNewChild && !Widget.canUpdate(widget.child!, _currentEntry!.widgetChild)) {
      // Child has changed, fade current entry out and add new entry.
      _childNumber += 1;
      _addEntryForNewChild(animate: true);
    } else if (_currentEntry != null) {
      assert(hasOldChild && hasNewChild);
      assert(Widget.canUpdate(widget.child!, _currentEntry!.widgetChild));
      // Child has been updated. Make sure we update the child widget and
      // transition in _currentEntry even though we're not going to start a new
      // animation, but keep the key from the previous transition so that we
      // update the transition instead of replacing it.
      _currentEntry!.widgetChild = widget.child!;
      _updateTransitionForEntry(_currentEntry!); // uses entry.widgetChild
      _markChildWidgetCacheAsDirty();
    }
  }

  void _addEntryForNewChild({required bool animate}) {
    assert(animate || _currentEntry == null);
    if (_currentEntry != null) {
      assert(animate);
      assert(!_outgoingEntries.contains(_currentEntry));
      _outgoingEntries.add(_currentEntry!);
      _currentEntry!.controller.reverse();
      _markChildWidgetCacheAsDirty();
      _currentEntry = null;
    }
    if (widget.child == null) {
      return;
    }
    final AnimationController controller = AnimationController(
      duration: widget.duration,
      reverseDuration: widget.reverseDuration,
      vsync: this,
    );
    final CurvedAnimation animation = CurvedAnimation(
      parent: controller,
      curve: widget.switchInCurve,
      reverseCurve: widget.switchOutCurve,
    );
    _currentEntry = _newEntry(
      child: widget.child!,
      controller: controller,
      animation: animation,
      builder: widget.transitionBuilder,
    );
    if (animate) {
      controller.forward();
    } else {
      assert(_outgoingEntries.isEmpty);
      controller.value = 1.0;
    }
  }

  _ChildEntry _newEntry({
    required Widget child,
    required AnimatedSwitcherTransitionBuilder builder,
    required AnimationController controller,
    required CurvedAnimation animation,
  }) {
    final _ChildEntry entry = _ChildEntry(
      widgetChild: child,
      transition: KeyedSubtree.wrap(builder(child, animation), _childNumber),
      animation: animation,
      controller: controller,
    );
    animation.addStatusListener((AnimationStatus status) {
      if (status.isDismissed) {
        setState(() {
          assert(mounted);
          assert(_outgoingEntries.contains(entry));
          _outgoingEntries.remove(entry);
          _markChildWidgetCacheAsDirty();
        });
        controller.dispose();
        animation.dispose();
      }
    });
    return entry;
  }

  void _markChildWidgetCacheAsDirty() {
    _outgoingWidgets = null;
  }

  void _updateTransitionForEntry(_ChildEntry entry) {
    entry.transition = KeyedSubtree(
      key: entry.transition.key,
      child: widget.transitionBuilder(entry.widgetChild, entry.animation),
    );
  }

  void _rebuildOutgoingWidgetsIfNeeded() {
    _outgoingWidgets ??= List<Widget>.unmodifiable(
      _outgoingEntries.map<Widget>((_ChildEntry entry) => entry.transition),
    );
    assert(_outgoingEntries.length == _outgoingWidgets!.length);
    assert(_outgoingEntries.isEmpty || _outgoingEntries.last.transition == _outgoingWidgets!.last);
  }

  @override
  void dispose() {
    _currentEntry?.controller.dispose();
    _currentEntry?.animation.dispose();
    for (final _ChildEntry entry in _outgoingEntries) {
      entry.controller.dispose();
      entry.animation.dispose();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    _rebuildOutgoingWidgetsIfNeeded();
    return widget.layoutBuilder(
      _currentEntry?.transition,
      _outgoingWidgets!
          .where((Widget outgoing) => outgoing.key != _currentEntry?.transition.key)
          .toSet()
          .toList(),
    );
  }
}
