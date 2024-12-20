// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/animation.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'async.dart';
/// @docImport 'notification_listener.dart';
/// @docImport 'text.dart';
/// @docImport 'transitions.dart';
/// @docImport 'tween_animation_builder.dart';
library;

import 'package:flutter/foundation.dart';

import 'framework.dart';

/// Builds a [Widget] when given a concrete value of a [ValueListenable<T>].
///
/// If the `child` parameter provided to the [ValueListenableBuilder] is not
/// null, the same `child` widget is passed back to this [ValueWidgetBuilder]
/// and should typically be incorporated in the returned widget tree.
///
/// See also:
///
///  * [ValueListenableBuilder], a widget which invokes this builder each time
///    a [ValueListenable] changes value.
typedef ValueWidgetBuilder<T> = Widget Function(BuildContext context, T value, Widget? child);

/// A widget whose content stays synced with a [ValueListenable].
///
/// Given a [ValueListenable<T>] and a [builder] which builds widgets from
/// concrete values of `T`, this class will automatically register itself as a
/// listener of the [ValueListenable] and call the [builder] with updated values
/// when the value changes.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=s-ZG-jS5QHQ}
///
/// ## Performance optimizations
///
/// If your [builder] function contains a subtree that does not depend on the
/// value of the [ValueListenable], it's more efficient to build that subtree
/// once instead of rebuilding it on every animation tick.
///
/// If you pass the pre-built subtree as the [child] parameter, the
/// [ValueListenableBuilder] will pass it back to your [builder] function so
/// that you can incorporate it into your build.
///
/// Using this pre-built child is entirely optional, but can improve
/// performance significantly in some cases and is therefore a good practice.
///
/// {@tool dartpad}
/// This sample shows how you could use a [ValueListenableBuilder] instead of
/// setting state on the whole [Scaffold] in a counter app.
///
/// ** See code in examples/api/lib/widgets/value_listenable_builder/value_listenable_builder.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedBuilder], which also triggers rebuilds from a [Listenable]
///    without passing back a specific value from a [ValueListenable].
///  * [NotificationListener], which lets you rebuild based on [Notification]
///    coming from its descendant widgets rather than a [ValueListenable] that
///    you have a direct reference to.
///  * [StreamBuilder], where a builder can depend on a [Stream] rather than
///    a [ValueListenable] for more advanced use cases.
///  * [TweenAnimationBuilder], which can animate values in a widget based on a
///    [Tween].
class ValueListenableBuilder<T> extends StatefulWidget {
  /// Creates a [ValueListenableBuilder].
  ///
  /// The [child] is optional but is good practice to use if part of the widget
  /// subtree does not depend on the value of the [valueListenable].
  const ValueListenableBuilder({
    super.key,
    required this.valueListenable,
    required this.builder,
    this.child,
  });

  /// The [ValueListenable] whose value you depend on in order to build.
  ///
  /// This widget does not ensure that the [ValueListenable]'s value is not
  /// null, therefore your [builder] may need to handle null values.
  final ValueListenable<T> valueListenable;

  /// A [ValueWidgetBuilder] which builds a widget depending on the
  /// [valueListenable]'s value.
  ///
  /// Can incorporate a [valueListenable] value-independent widget subtree
  /// from the [child] parameter into the returned widget tree.
  final ValueWidgetBuilder<T> builder;

  /// A [valueListenable]-independent widget which is passed back to the [builder].
  ///
  /// This argument is optional and can be null if the entire widget subtree the
  /// [builder] builds depends on the value of the [valueListenable]. For
  /// example, in the case where the [valueListenable] is a [String] and the
  /// [builder] returns a [Text] widget with the current [String] value, there
  /// would be no useful [child].
  final Widget? child;

  @override
  State<StatefulWidget> createState() => _ValueListenableBuilderState<T>();
}

class _ValueListenableBuilderState<T> extends State<ValueListenableBuilder<T>> {
  late T value;

  @override
  void initState() {
    super.initState();
    value = widget.valueListenable.value;
    widget.valueListenable.addListener(_valueChanged);
  }

  @override
  void didUpdateWidget(ValueListenableBuilder<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.valueListenable != widget.valueListenable) {
      oldWidget.valueListenable.removeListener(_valueChanged);
      value = widget.valueListenable.value;
      widget.valueListenable.addListener(_valueChanged);
    }
  }

  @override
  void dispose() {
    widget.valueListenable.removeListener(_valueChanged);
    super.dispose();
  }

  void _valueChanged() {
    setState(() {
      value = widget.valueListenable.value;
    });
  }

  @override
  Widget build(BuildContext context) {
    return widget.builder(context, value, widget.child);
  }
}
