// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/foundation.dart';
///
/// @docImport 'inherited_notifier.dart';
library;

import 'dart:collection';

import 'framework.dart';

/// An [InheritedWidget] that's intended to be used as the base class for models
/// whose dependents may only depend on one part or "aspect" of the overall
/// model.
///
/// An inherited widget's dependents are unconditionally rebuilt when the
/// inherited widget changes per [InheritedWidget.updateShouldNotify]. This
/// widget is similar except that dependents aren't rebuilt unconditionally.
///
/// Widgets that depend on an [InheritedModel] qualify their dependence with a
/// value that indicates what "aspect" of the model they depend on. When the
/// model is rebuilt, dependents will also be rebuilt, but only if there was a
/// change in the model that corresponds to the aspect they provided.
///
/// The type parameter `T` is the type of the model aspect objects.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=ml5uefGgkaA}
///
/// Widgets create a dependency on an [InheritedModel] with a static method:
/// [InheritedModel.inheritFrom]. This method's `context` parameter defines the
/// subtree that will be rebuilt when the model changes. Typically the
/// `inheritFrom` method is called from a model-specific static `maybeOf` or
/// `of` methods, a convention that is present in many Flutter framework classes
/// which look things up. For example:
///
/// ```dart
/// class MyModel extends InheritedModel<String> {
///   const MyModel({super.key, required super.child});
///
///   // ...
///   static MyModel? maybeOf(BuildContext context, [String? aspect]) {
///     return InheritedModel.inheritFrom<MyModel>(context, aspect: aspect);
///   }
///
///   // ...
///   static MyModel of(BuildContext context, [String? aspect]) {
///     final MyModel? result = maybeOf(context, aspect);
///     assert(result != null, 'Unable to find an instance of MyModel...');
///     return result!;
///   }
/// }
/// ```
///
/// Calling `MyModel.of(context, 'foo')` or `MyModel.maybeOf(context,
/// 'foo')` means that `context` should only be rebuilt when the `foo` aspect of
/// `MyModel` changes. If the `aspect` is null, then the model supports all
/// aspects.
///
/// {@tool snippet}
/// When the inherited model is rebuilt the [updateShouldNotify] and
/// [updateShouldNotifyDependent] methods are used to decide what should be
/// rebuilt. If [updateShouldNotify] returns true, then the inherited model's
/// [updateShouldNotifyDependent] method is tested for each dependent and the
/// set of aspect objects it depends on. The [updateShouldNotifyDependent]
/// method must compare the set of aspect dependencies with the changes in the
/// model itself. For example:
///
/// ```dart
/// class ABModel extends InheritedModel<String> {
///   const ABModel({
///    super.key,
///    this.a,
///    this.b,
///    required super.child,
///   });
///
///   final int? a;
///   final int? b;
///
///   @override
///   bool updateShouldNotify(ABModel oldWidget) {
///     return a != oldWidget.a || b != oldWidget.b;
///   }
///
///   @override
///   bool updateShouldNotifyDependent(ABModel oldWidget, Set<String> dependencies) {
///     return (a != oldWidget.a && dependencies.contains('a'))
///       || (b != oldWidget.b && dependencies.contains('b'));
///   }
///
///   // ...
/// }
/// ```
/// {@end-tool}
///
/// In the previous example the dependencies checked by
/// [updateShouldNotifyDependent] are just the aspect strings passed to
/// `dependOnInheritedWidgetOfExactType`. They're represented as a [Set] because
/// one Widget can depend on more than one aspect of the model. If a widget
/// depends on the model but doesn't specify an aspect, then changes in the
/// model will cause the widget to be rebuilt unconditionally.
///
/// {@tool dartpad}
/// This example shows how to implement [InheritedModel] to rebuild a widget
/// based on a qualified dependence. When tapped on the "Resize Logo" button
/// only the logo widget is rebuilt while the background widget remains
/// unaffected.
///
/// ** See code in examples/api/lib/widgets/inherited_model/inherited_model.0.dart **
/// {@end-tool}
///
/// See also:
///
/// * [InheritedWidget], an inherited widget that only notifies dependents when
///   its value is different.
/// * [InheritedNotifier], an inherited widget whose value can be a
///   [Listenable], and which will notify dependents whenever the value sends
///   notifications.
abstract class InheritedModel<T> extends InheritedWidget {
  /// Creates an inherited widget that supports dependencies qualified by
  /// "aspects", i.e. a descendant widget can indicate that it should
  /// only be rebuilt if a specific aspect of the model changes.
  const InheritedModel({super.key, required super.child});

  @override
  InheritedModelElement<T> createElement() => InheritedModelElement<T>(this);

  /// Return true if the changes between this model and [oldWidget] match any
  /// of the [dependencies].
  @protected
  bool updateShouldNotifyDependent(covariant InheritedModel<T> oldWidget, Set<T> dependencies);

  /// Returns true if this model supports the given [aspect].
  ///
  /// Returns true by default: this model supports all aspects.
  ///
  /// Subclasses may override this method to indicate that they do not support
  /// all model aspects. This is typically done when a model can be used
  /// to "shadow" some aspects of an ancestor.
  @protected
  bool isSupportedAspect(Object aspect) => true;

  // The [result] will be a list of all of context's type T ancestors concluding
  // with the one that supports the specified model [aspect].
  static void _findModels<T extends InheritedModel<Object>>(
    BuildContext context,
    Object aspect,
    List<InheritedElement> results,
  ) {
    final InheritedElement? model = context.getElementForInheritedWidgetOfExactType<T>();
    if (model == null) {
      return;
    }

    results.add(model);

    assert(model.widget is T);
    final T modelWidget = model.widget as T;
    if (modelWidget.isSupportedAspect(aspect)) {
      return;
    }

    Element? modelParent;
    model.visitAncestorElements((Element ancestor) {
      modelParent = ancestor;
      return false;
    });
    if (modelParent == null) {
      return;
    }

    _findModels<T>(modelParent!, aspect, results);
  }

  /// Makes [context] dependent on the specified [aspect] of an [InheritedModel]
  /// of type T.
  ///
  /// When the given [aspect] of the model changes, the [context] will be
  /// rebuilt. The [updateShouldNotifyDependent] method must determine if a
  /// change in the model widget corresponds to an [aspect] value.
  ///
  /// The dependencies created by this method target all [InheritedModel] ancestors
  /// of type T up to and including the first one for which [isSupportedAspect]
  /// returns true.
  ///
  /// If [aspect] is null this method is the same as
  /// `context.dependOnInheritedWidgetOfExactType<T>()`.
  ///
  /// If no ancestor of type T exists, null is returned.
  static T? inheritFrom<T extends InheritedModel<Object>>(BuildContext context, {Object? aspect}) {
    if (aspect == null) {
      return context.dependOnInheritedWidgetOfExactType<T>();
    }

    // Create a dependency on all of the type T ancestor models up until
    // a model is found for which isSupportedAspect(aspect) is true.
    final List<InheritedElement> models = <InheritedElement>[];
    _findModels<T>(context, aspect, models);
    if (models.isEmpty) {
      return null;
    }

    final InheritedElement lastModel = models.last;
    for (final InheritedElement model in models) {
      final T value = context.dependOnInheritedElement(model, aspect: aspect) as T;
      if (model == lastModel) {
        return value;
      }
    }

    assert(false);
    return null;
  }
}

/// An [Element] that uses a [InheritedModel] as its configuration.
class InheritedModelElement<T> extends InheritedElement {
  /// Creates an element that uses the given widget as its configuration.
  InheritedModelElement(InheritedModel<T> super.widget);

  @override
  void updateDependencies(Element dependent, Object? aspect) {
    final Set<T>? dependencies = getDependencies(dependent) as Set<T>?;
    if (dependencies != null && dependencies.isEmpty) {
      return;
    }

    if (aspect == null) {
      setDependencies(dependent, HashSet<T>());
    } else {
      assert(aspect is T);
      setDependencies(dependent, (dependencies ?? HashSet<T>())..add(aspect as T));
    }
  }

  @override
  void notifyDependent(InheritedModel<T> oldWidget, Element dependent) {
    final Set<T>? dependencies = getDependencies(dependent) as Set<T>?;
    if (dependencies == null) {
      return;
    }
    if (dependencies.isEmpty ||
        (widget as InheritedModel<T>).updateShouldNotifyDependent(oldWidget, dependencies)) {
      dependent.didChangeDependencies();
    }
  }
}
