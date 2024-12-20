// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'routes.dart';
/// @docImport 'scroll_controller.dart';
/// @docImport 'scroll_position.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'scrollable.dart';
/// @docImport 'single_child_scroll_view.dart';
library;

import 'package:flutter/foundation.dart';

import 'framework.dart';

// Examples can assume:
// late BuildContext context;

/// A [Key] that can be used to persist the widget state in storage after the
/// destruction and will be restored when recreated.
///
/// Each key with its value plus the ancestor chain of other [PageStorageKey]s
/// need to be unique within the widget's closest ancestor [PageStorage]. To
/// make it possible for a saved value to be found when a widget is recreated,
/// the key's value must not be objects whose identity will change each time the
/// widget is created.
///
/// See also:
///
///  * [PageStorage], which manages the data storage for widgets using
///    [PageStorageKey]s.
class PageStorageKey<T> extends ValueKey<T> {
  /// Creates a [ValueKey] that defines where [PageStorage] values will be saved.
  const PageStorageKey(super.value);
}

@immutable
class _StorageEntryIdentifier {
  const _StorageEntryIdentifier(this.keys);

  final List<PageStorageKey<dynamic>> keys;

  bool get isNotEmpty => keys.isNotEmpty;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _StorageEntryIdentifier &&
        listEquals<PageStorageKey<dynamic>>(other.keys, keys);
  }

  @override
  int get hashCode => Object.hashAll(keys);

  @override
  String toString() {
    return 'StorageEntryIdentifier(${keys.join(":")})';
  }
}

/// A storage bucket associated with a page in an app.
///
/// Useful for storing per-page state that persists across navigations from one
/// page to another.
class PageStorageBucket {
  static bool _maybeAddKey(BuildContext context, List<PageStorageKey<dynamic>> keys) {
    final Widget widget = context.widget;
    final Key? key = widget.key;
    if (key is PageStorageKey) {
      keys.add(key);
    }
    return widget is! PageStorage;
  }

  List<PageStorageKey<dynamic>> _allKeys(BuildContext context) {
    final List<PageStorageKey<dynamic>> keys = <PageStorageKey<dynamic>>[];
    if (_maybeAddKey(context, keys)) {
      context.visitAncestorElements((Element element) {
        return _maybeAddKey(element, keys);
      });
    }
    return keys;
  }

  _StorageEntryIdentifier _computeIdentifier(BuildContext context) {
    return _StorageEntryIdentifier(_allKeys(context));
  }

  Map<Object, dynamic>? _storage;

  /// Write the given data into this page storage bucket using the
  /// specified identifier or an identifier computed from the given context.
  /// The computed identifier is based on the [PageStorageKey]s
  /// found in the path from context to the [PageStorage] widget that
  /// owns this page storage bucket.
  ///
  /// If an explicit identifier is not provided and no [PageStorageKey]s
  /// are found, then the `data` is not saved.
  void writeState(BuildContext context, dynamic data, {Object? identifier}) {
    _storage ??= <Object, dynamic>{};
    if (identifier != null) {
      _storage![identifier] = data;
    } else {
      final _StorageEntryIdentifier contextIdentifier = _computeIdentifier(context);
      if (contextIdentifier.isNotEmpty) {
        _storage![contextIdentifier] = data;
      }
    }
  }

  /// Read given data from into this page storage bucket using the specified
  /// identifier or an identifier computed from the given context.
  /// The computed identifier is based on the [PageStorageKey]s
  /// found in the path from context to the [PageStorage] widget that
  /// owns this page storage bucket.
  ///
  /// If an explicit identifier is not provided and no [PageStorageKey]s
  /// are found, then null is returned.
  dynamic readState(BuildContext context, {Object? identifier}) {
    if (_storage == null) {
      return null;
    }
    if (identifier != null) {
      return _storage![identifier];
    }
    final _StorageEntryIdentifier contextIdentifier = _computeIdentifier(context);
    return contextIdentifier.isNotEmpty ? _storage![contextIdentifier] : null;
  }
}

/// Establish a subtree in which widgets can opt into persisting states after
/// being destroyed.
///
/// [PageStorage] is used to save and restore values that can outlive the widget.
/// For example, when multiple pages are grouped in tabs, when a page is
/// switched out, its widget is destroyed and its state is lost. By adding a
/// [PageStorage] at the root and adding a [PageStorageKey] to each page, some of the
/// page's state (e.g. the scroll position of a [Scrollable] widget) will be stored
/// automatically in its closest ancestor [PageStorage], and restored when it's
/// switched back.
///
/// Usually you don't need to explicitly use a [PageStorage], since it's already
/// included in routes.
///
/// [PageStorageKey] is used by [Scrollable] if [ScrollController.keepScrollOffset]
/// is enabled to save their [ScrollPosition]s. When more than one scrollable
/// ([ListView], [SingleChildScrollView], [TextField], etc.) appears within the
/// widget's closest ancestor [PageStorage] (such as within the same route), to
/// save all of their positions independently, one must give each of them unique
/// [PageStorageKey]s, or set the `keepScrollOffset` property of some such
/// widgets to false to prevent saving.
///
/// {@tool dartpad}
/// This sample shows how to explicitly use a [PageStorage] to
/// store the states of its children pages. Each page includes a scrollable
/// list, whose position is preserved when switching between the tabs thanks to
/// the help of [PageStorageKey].
///
/// ** See code in examples/api/lib/widgets/page_storage/page_storage.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [ModalRoute], which includes this class.
class PageStorage extends StatelessWidget {
  /// Creates a widget that provides a storage bucket for its descendants.
  const PageStorage({super.key, required this.bucket, required this.child});

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The page storage bucket to use for this subtree.
  final PageStorageBucket bucket;

  /// The [PageStorageBucket] from the closest instance of a [PageStorage]
  /// widget that encloses the given context.
  ///
  /// Returns null if none exists.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// PageStorageBucket? bucket = PageStorage.of(context);
  /// ```
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// See also:
  ///
  /// * [PageStorage.of], which is similar to this method, but
  ///   asserts if no [PageStorage] ancestor is found.
  static PageStorageBucket? maybeOf(BuildContext context) {
    final PageStorage? widget = context.findAncestorWidgetOfExactType<PageStorage>();
    return widget?.bucket;
  }

  /// The [PageStorageBucket] from the closest instance of a [PageStorage]
  /// widget that encloses the given context.
  ///
  /// If no ancestor is found, this method will assert in debug mode, and throw
  /// an exception in release mode.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// PageStorageBucket bucket = PageStorage.of(context);
  /// ```
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// See also:
  ///
  /// * [PageStorage.maybeOf], which is similar to this method, but
  ///   returns null if no [PageStorage] ancestor is found.
  static PageStorageBucket of(BuildContext context) {
    final PageStorageBucket? bucket = maybeOf(context);
    assert(() {
      if (bucket == null) {
        throw FlutterError(
          'PageStorage.of() was called with a context that does not contain a '
          'PageStorage widget.\n'
          'No PageStorage widget ancestor could be found starting from the '
          'context that was passed to PageStorage.of(). This can happen '
          'because you are using a widget that looks for a PageStorage '
          'ancestor, but no such ancestor exists.\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return bucket!;
  }

  @override
  Widget build(BuildContext context) => child;
}
