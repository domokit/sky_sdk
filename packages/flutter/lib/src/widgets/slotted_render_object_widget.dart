// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'framework.dart';

/// A superclass for [RenderObjectWidget]s that configure [RenderObject]
/// subclasses that organize their children in different slots.
///
/// Implementers of this mixin have to provide the list of available slots by
/// overriding [slots]. The list of slots must never change for a given class
/// implementing this mixin. In the common case, [Enum] values are used as slots
/// and [slots] is typically implemented to return the value of the enum's
/// `values` getter.
///
/// Furthermore, [childForSlot] must be implemented to return the current
/// widget configuration for a given slot.
///
/// The [RenderObject] returned by [createRenderObject] and updated by
/// [updateRenderObject] must implement [SlottedContainerRenderObjectMixin].
///
/// The type parameter `SlotType` is the type for the slots to be used by this
/// [RenderObjectWidget] and the [RenderObject] it configures. In the typical
/// case, `SlotType` is an [Enum] type.
///
/// The type parameter `ChildType` is the type used for the [RenderObject] children
/// (e.g. [RenderBox] or [RenderSliver]). In the typical case, `ChildType` is
/// [RenderBox]. This class does not support having different kinds of children
/// for different slots.
///
/// {@tool dartpad}
/// This example uses the [SlottedMultiChildRenderObjectWidget] in
/// combination with the [SlottedContainerRenderObjectMixin] to implement a
/// widget that provides two slots: topLeft and bottomRight. The widget arranges
/// the children in those slots diagonally.
///
/// ** See code in examples/api/lib/widgets/slotted_render_object_widget/slotted_multi_child_render_object_widget_mixin.0.dart **
/// {@end-tool}
///
/// See also:
///
///   * [MultiChildRenderObjectWidget], which configures a [RenderObject]
///     with a single list of children.
///   * [ListTile], which uses [SlottedMultiChildRenderObjectWidget] in its
///     internal (private) implementation.
abstract class SlottedMultiChildRenderObjectWidget<SlotType, ChildType extends RenderObject>
    extends RenderObjectWidget
    with SlottedMultiChildRenderObjectWidgetMixin<SlotType, ChildType> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const SlottedMultiChildRenderObjectWidget({super.key});
}

/// A mixin version of [SlottedMultiChildRenderObjectWidget].
///
/// This mixin provides the same logic as extending
/// [SlottedMultiChildRenderObjectWidget] directly.
///
/// It was deprecated to simplify the process of creating slotted widgets.
@Deprecated(
  'Extend SlottedMultiChildRenderObjectWidget instead of mixing in SlottedMultiChildRenderObjectWidgetMixin. '
  'This feature was deprecated after v3.10.0-1.5.pre.',
)
mixin SlottedMultiChildRenderObjectWidgetMixin<SlotType, ChildType extends RenderObject>
    on RenderObjectWidget {
  /// Returns a list of all available slots.
  ///
  /// The list of slots must be static and must never change for a given class
  /// implementing this mixin.
  ///
  /// Typically, an [Enum] is used to identify the different slots. In that case
  /// this getter can be implemented by returning what the `values` getter
  /// of the enum used returns.
  @protected
  Iterable<SlotType> get slots;

  /// Returns the widget that is currently occupying the provided `slot`.
  ///
  /// The [RenderObject] configured by this class will be configured to have
  /// the [RenderObject] produced by the returned [Widget] in the provided
  /// `slot`.
  @protected
  Widget? childForSlot(SlotType slot);

  @override
  SlottedContainerRenderObjectMixin<SlotType, ChildType> createRenderObject(BuildContext context);

  @override
  void updateRenderObject(
    BuildContext context,
    SlottedContainerRenderObjectMixin<SlotType, ChildType> renderObject,
  );

  @override
  SlottedRenderObjectElement<SlotType, ChildType> createElement() =>
      SlottedRenderObjectElement<SlotType, ChildType>(this);
}

/// Mixin for a [RenderObject] configured by a [SlottedMultiChildRenderObjectWidget].
///
/// The [RenderObject] child currently occupying a given slot can be obtained by
/// calling [childForSlot].
///
/// Implementers may consider overriding [children] to return the children
/// of this render object in a consistent order (e.g. hit test order).
///
/// The type parameter `SlotType` is the type for the slots to be used by this
/// [RenderObject] and the [SlottedMultiChildRenderObjectWidget] it was
/// configured by. In the typical case, `SlotType` is an [Enum] type.
///
/// The type parameter `ChildType` is the type of [RenderObject] used for the children
/// (e.g. [RenderBox] or [RenderSliver]). In the typical case, `ChildType` is
/// [RenderBox]. This mixin does not support having different kinds of children
/// for different slots.
///
/// See [SlottedMultiChildRenderObjectWidget] for example code showcasing how
/// this mixin is used in combination with [SlottedMultiChildRenderObjectWidget].
///
/// See also:
///
///  * [ContainerRenderObjectMixin], which organizes its children in a single
///    list.
mixin SlottedContainerRenderObjectMixin<SlotType, ChildType extends RenderObject> on RenderObject {
  /// Returns the [RenderObject] child that is currently occupying the provided
  /// `slot`.
  ///
  /// Returns null if no [RenderObject] is configured for the given slot.
  @protected
  ChildType? childForSlot(SlotType slot) => _slotToChild[slot];

  /// Returns an [Iterable] of all non-null children.
  ///
  /// This getter is used by the default implementation of [attach], [detach],
  /// [redepthChildren], [visitChildren], and [debugDescribeChildren] to iterate
  /// over the children of this [RenderObject]. The base implementation makes no
  /// guarantee about the order in which the children are returned. Subclasses
  /// for which the child order is important should override this getter and
  /// return the children in the desired order.
  @protected
  Iterable<ChildType> get children => _slotToChild.values;

  /// Returns the debug name for a given `slot`.
  ///
  /// This method is called by [debugDescribeChildren] for each slot that is
  /// currently occupied by a child to obtain a name for that slot for debug
  /// outputs.
  ///
  /// The default implementation calls [EnumName.name] on `slot` if it is an
  /// [Enum] value and `toString` if it is not.
  @protected
  String debugNameForSlot(SlotType slot) {
    if (slot is Enum) {
      return slot.name;
    }
    return slot.toString();
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    for (final ChildType child in children) {
      child.attach(owner);
    }
  }

  @override
  void detach() {
    super.detach();
    for (final ChildType child in children) {
      child.detach();
    }
  }

  @override
  void redepthChildren() {
    children.forEach(redepthChild);
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    children.forEach(visitor);
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    final List<DiagnosticsNode> value = <DiagnosticsNode>[];
    final Map<ChildType, SlotType> childToSlot = Map<ChildType, SlotType>.fromIterables(
      _slotToChild.values,
      _slotToChild.keys,
    );
    for (final ChildType child in children) {
      _addDiagnostics(child, value, debugNameForSlot(childToSlot[child] as SlotType));
    }
    return value;
  }

  void _addDiagnostics(ChildType child, List<DiagnosticsNode> value, String name) {
    value.add(child.toDiagnosticsNode(name: name));
  }

  final Map<SlotType, ChildType> _slotToChild = <SlotType, ChildType>{};

  void _setChild(ChildType? child, SlotType slot) {
    final ChildType? oldChild = _slotToChild[slot];
    if (oldChild != null) {
      dropChild(oldChild);
      _slotToChild.remove(slot);
    }
    if (child != null) {
      _slotToChild[slot] = child;
      adoptChild(child);
    }
  }

  void _moveChild(ChildType child, SlotType slot, SlotType oldSlot) {
    assert(slot != oldSlot);
    final ChildType? oldChild = _slotToChild[oldSlot];
    if (oldChild == child) {
      _setChild(null, oldSlot);
    }
    _setChild(child, slot);
  }
}

/// Element used by the [SlottedMultiChildRenderObjectWidget].
class SlottedRenderObjectElement<SlotType, ChildType extends RenderObject>
    extends RenderObjectElement {
  /// Creates an element that uses the given widget as its configuration.
  SlottedRenderObjectElement(
    SlottedMultiChildRenderObjectWidgetMixin<SlotType, ChildType> super.widget,
  );

  Map<SlotType, Element> _slotToChild = <SlotType, Element>{};
  Map<Key, Element> _keyedChildren = <Key, Element>{};

  @override
  SlottedContainerRenderObjectMixin<SlotType, ChildType> get renderObject =>
      super.renderObject as SlottedContainerRenderObjectMixin<SlotType, ChildType>;

  @override
  void visitChildren(ElementVisitor visitor) {
    _slotToChild.values.forEach(visitor);
  }

  @override
  void forgetChild(Element child) {
    assert(_slotToChild.containsValue(child));
    assert(child.slot is SlotType);
    assert(_slotToChild.containsKey(child.slot));
    _slotToChild.remove(child.slot);
    super.forgetChild(child);
  }

  @override
  void mount(Element? parent, Object? newSlot) {
    super.mount(parent, newSlot);
    _updateChildren();
  }

  @override
  void update(SlottedMultiChildRenderObjectWidgetMixin<SlotType, ChildType> newWidget) {
    super.update(newWidget);
    assert(widget == newWidget);
    _updateChildren();
  }

  List<SlotType>? _debugPreviousSlots;

  void _updateChildren() {
    final SlottedMultiChildRenderObjectWidgetMixin<SlotType, ChildType>
    slottedMultiChildRenderObjectWidgetMixin =
        widget as SlottedMultiChildRenderObjectWidgetMixin<SlotType, ChildType>;
    assert(() {
      _debugPreviousSlots ??= slottedMultiChildRenderObjectWidgetMixin.slots.toList();
      return listEquals(
        _debugPreviousSlots,
        slottedMultiChildRenderObjectWidgetMixin.slots.toList(),
      );
    }(), '${widget.runtimeType}.slots must not change.');
    assert(
      slottedMultiChildRenderObjectWidgetMixin.slots.toSet().length ==
          slottedMultiChildRenderObjectWidgetMixin.slots.length,
      'slots must be unique',
    );

    final Map<Key, Element> oldKeyedElements = _keyedChildren;
    _keyedChildren = <Key, Element>{};
    final Map<SlotType, Element> oldSlotToChild = _slotToChild;
    _slotToChild = <SlotType, Element>{};

    Map<Key, List<Element>>? debugDuplicateKeys;

    for (final SlotType slot in slottedMultiChildRenderObjectWidgetMixin.slots) {
      final Widget? widget = slottedMultiChildRenderObjectWidgetMixin.childForSlot(slot);
      final Key? newWidgetKey = widget?.key;

      final Element? oldSlotChild = oldSlotToChild[slot];
      final Element? oldKeyChild = oldKeyedElements[newWidgetKey];

      // Try to find the slot for the correct Element that `widget` should update.
      // If key matching fails, resort to `oldSlotChild` from the same slot.
      final Element? fromElement;
      if (oldKeyChild != null) {
        fromElement = oldSlotToChild.remove(oldKeyChild.slot as SlotType);
      } else if (oldSlotChild?.widget.key == null) {
        fromElement = oldSlotToChild.remove(slot);
      } else {
        // The only case we can't use `oldSlotChild` is when its widget has a key.
        assert(oldSlotChild!.widget.key != newWidgetKey);
        fromElement = null;
      }
      final Element? newChild = updateChild(fromElement, widget, slot);

      if (newChild != null) {
        _slotToChild[slot] = newChild;

        if (newWidgetKey != null) {
          assert(() {
            final Element? existingElement = _keyedChildren[newWidgetKey];
            if (existingElement != null) {
              (debugDuplicateKeys ??= <Key, List<Element>>{})
                  .putIfAbsent(newWidgetKey, () => <Element>[existingElement])
                  .add(newChild);
            }
            return true;
          }());
          _keyedChildren[newWidgetKey] = newChild;
        }
      }
    }
    oldSlotToChild.values.forEach(deactivateChild);
    assert(_debugDuplicateKeys(debugDuplicateKeys));
    assert(
      _keyedChildren.values.every(_slotToChild.values.contains),
      '_keyedChildren ${_keyedChildren.values} should be a subset of ${_slotToChild.values}',
    );
  }

  bool _debugDuplicateKeys(Map<Key, List<Element>>? debugDuplicateKeys) {
    if (debugDuplicateKeys == null) {
      return true;
    }
    for (final MapEntry<Key, List<Element>> duplicateKey in debugDuplicateKeys.entries) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('Multiple widgets used the same key in ${widget.runtimeType}.'),
        ErrorDescription(
          'The key ${duplicateKey.key} was used by multiple widgets. The offending widgets were:\n',
        ),
        for (final Element element in duplicateKey.value) ErrorDescription('  - $element\n'),
        ErrorDescription(
          'A key can only be specified on one widget at a time in the same parent widget.',
        ),
      ]);
    }
    return true;
  }

  @override
  void insertRenderObjectChild(ChildType child, SlotType slot) {
    renderObject._setChild(child, slot);
    assert(renderObject._slotToChild[slot] == child);
  }

  @override
  void removeRenderObjectChild(ChildType child, SlotType slot) {
    if (renderObject._slotToChild[slot] == child) {
      renderObject._setChild(null, slot);
      assert(renderObject._slotToChild[slot] == null);
    }
  }

  @override
  void moveRenderObjectChild(ChildType child, SlotType oldSlot, SlotType newSlot) {
    renderObject._moveChild(child, newSlot, oldSlot);
  }
}
