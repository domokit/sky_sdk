// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import 'semantics.dart';

/// The method used to represend a label of a leaf node in the DOM.
///
/// This is required by some screen readers and web crawlers.
///
/// Container nodes only use `aria-label`, even if [domText] is chosen. This is
/// because screen readers treat container nodes as "groups" of other nodes, and
/// respect the `aria-label` without a [DomText] node. Crawlers typically do not
/// need this information, as they primarily scan visible text, which is
/// communicated in semantics as leaf text and heading nodes.
enum LabelRepresentation {
  /// Represents the label as an `aria-label` attribute.
  ///
  /// This representation is the most efficient as all it does is pass a string
  /// to the browser that does not incur any DOM costs.
  ///
  /// The drawback of this representation is that it is not compatible with most
  /// web crawlers, and for some ARIA roles (including the implicit "generic"
  /// role) JAWS on Windows. However, this role is still the most common, as it
  /// applies to all container nodes, and many ARIA roles (e.g. checkboxes,
  /// radios, scrollables, sliders).
  ariaLabel,

  /// Represents the label as a [DomText] node.
  ///
  /// This is the second fastest way to represent a label in the DOM. It has a
  /// small cost because the browser lays out the text (in addition to Flutter
  /// having already done it).
  ///
  /// This representation is compatible with most web crawlers, and it is the
  /// best option for certain ARIA roles, such as buttons, links, and headings.
  domText,

  /// Represents the label as a sized span.
  ///
  /// This representation is the costliest as it uses an extra element that
  /// need to be laid out to compute the right size. It is compatible with most
  /// web crawlers, and it is the best options for certain ARIA roles, such as
  /// the implicit "generic" role used for plain text (not headings).
  sizedSpan;

  /// Creates the behavior for this label representation.
  LabelRepresentationBehavior createBehavior(SemanticRole owner) {
    return switch (this) {
      ariaLabel => AriaLabelRepresentation._(owner),
      domText => DomTextRepresentation._(owner),
      sizedSpan => SizedSpanRepresentation._(owner),
    };
  }
}

/// Provides a DOM behavior for a [LabelRepresentation].
abstract final class LabelRepresentationBehavior {
  LabelRepresentationBehavior(this.kind, this.owner);

  final LabelRepresentation kind;

  /// The role that this label representation is attached to.
  final SemanticRole owner;

  /// Convenience getter for the corresponding semantics object.
  SemanticsObject get semanticsObject => owner.semanticsObject;

  /// Updates the label displayed to the user.
  void update(String label);

  /// Removes the DOM associated with this label.
  ///
  /// This can happen when the representation is changed from one type to
  /// another.
  void cleanUp();

  /// The element that gets focus when [focusAsRouteDefault] is called.
  ///
  /// Each label behavior decides which element should be focused on based on
  /// its own bespoke DOM structure.
  DomElement get focusTarget;

  /// Move the accessibility focus to the element the carries the label assuming
  /// the node is not [Focusable].
  ///
  /// Since normally, plain text is not focusable (e.g. it doesn't have explicit
  /// or implicit `tabindex`), `tabindex` must be added artificially.
  ///
  /// Plain text nodes should not be focusable via keyboard or mouse. They are
  /// only focusable for the purposes of focusing the screen reader. To achieve
  /// this the -1 value is used.
  ///
  /// See also:
  ///
  /// https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes/tabindex
  void focusAsRouteDefault() {
    focusTarget.tabIndex = -1;
    focusTarget.focusWithoutScroll();
  }
}

/// Sets the label as `aria-label`.
///
/// Example:
///
///     <flt-semantics aria-label="Hello, World!"></flt-semantics>
final class AriaLabelRepresentation extends LabelRepresentationBehavior {
  AriaLabelRepresentation._(SemanticRole owner) : super(LabelRepresentation.ariaLabel, owner);

  String? _previousLabel;

  @override
  void update(String label) {
    if (label == _previousLabel) {
      return;
    }
    owner.setAttribute('aria-label', label);
  }

  @override
  void cleanUp() {
    owner.removeAttribute('aria-label');
  }

  // ARIA label does not introduce extra DOM elements, so focus should go to the
  // semantic node's host element.
  @override
  DomElement get focusTarget => owner.element;
}

/// Sets the label as text inside the DOM element.
///
/// Example:
///
///     <flt-semantics>Hello, World!</flt-semantics>
///
/// This representation is used when the ARIA role of the element already sizes
/// the element and therefore no extra sizing assistance is needed. If there is
/// no ARIA role set, or the role does not size the element, then the
/// [SizedSpanRepresentation] representation can be used.
final class DomTextRepresentation extends LabelRepresentationBehavior {
  DomTextRepresentation._(SemanticRole owner) : super(LabelRepresentation.domText, owner);

  DomText? _domText;
  String? _previousLabel;

  @override
  void update(String label) {
    if (label == _previousLabel) {
      return;
    }

    _domText?.remove();
    final DomText domText = domDocument.createTextNode(label);
    _domText = domText;
    semanticsObject.element.appendChild(domText);
  }

  @override
  void cleanUp() {
    _domText?.remove();
  }

  // DOM text does not introduce extra DOM elements, so focus should go to the
  // semantic node's host element.
  @override
  DomElement get focusTarget => owner.element;
}

/// A span queue for a size update.
typedef _QueuedSizeUpdate =
    ({
      // The span to be sized.
      SizedSpanRepresentation representation,

      // The desired size.
      ui.Size targetSize,
    });

/// The size of a span as measured in the DOM.
typedef _Measurement =
    ({
      // The span that was measured.
      SizedSpanRepresentation representation,

      // The measured size of the DOM element before the size adjustment.
      ui.Size domSize,

      // The size of the element that the screen reader should observe after the
      // size adjustment.
      ui.Size targetSize,
    });

/// Sets the label as the text of a `<span>` child element.
///
/// The span element is scaled to match the size of the semantic node.
///
/// Example:
///
///     <flt-semantics>
///       <span style="transform: scale(2, 2)">Hello, World!</span>
///     </flt-semantics>
///
/// Text scaling is used to control the size of the screen reader focus ring.
/// This is used for plain text nodes (e.g. paragraphs of text).
///
/// ## Why use scaling rather than another method?
///
/// Due to https://g-issues.chromium.org/issues/40875151?pli=1&authuser=0 and a
/// lack of an ARIA role for plain text nodes (expecially after the removal of
/// ARIA role "text" in WebKit, starting with Safari 17), there is no way to
/// customize the size of the screen reader focus ring for a plain text element.
/// The focus ring always tightly hugs the text itself. The following approaches
/// were tried, and all failed:
///
/// * `text-align` + dummy text to force text align to span the width of the
///   element. This does affect the screen reader focus size, but this method is
///   limited to width only. There's no way to control the height. Also, using
///   dummy text at the end feels extremely hacky, and risks failing due to
///   proprietary screen reader behaviors - they may not consistency react to
///   the dummy text (e.g. some may read it out loud).
/// * The following methods did not have the desired effect:
///   - Different `display` values.
///   - Adding visual/layout features to the element: border, outline, padding,
///     box-sizing, text-shadow.
/// * `role="text"` was used previously and worked, but only in Safari pre-17.
/// * `role="group"` sizes the element correctly, but breaks the message to the
///   user (reads "empty group", requires multi-step traversal).
/// * Adding `aria-hidden` contents to the element. This results in "group"
///   behavior.
/// * Use an existing non-text role, e.g. "heading". Sizes correctly, but breaks
///   the message (reads "heading").
final class SizedSpanRepresentation extends LabelRepresentationBehavior {
  SizedSpanRepresentation._(SemanticRole owner) : super(LabelRepresentation.sizedSpan, owner) {
    _domText.style
      // `inline-block` is needed for two reasons:
      // - It supports measuring the true size of the text. Pure `block` would
      //   disassociate the size of the text from the size of the element.
      // - It supports the `transform` and `transform-origin` properties. Pure
      //   `inline` does not support them.
      ..display = 'inline-block'
      // Do not wrap text based on parent constraints. Instead, to fit in the
      // parent's box the text will be scaled.
      ..whiteSpace = 'nowrap'
      // The origin of the coordinate system is the top-left corner of the
      // parent element.
      ..transformOrigin = '0 0 0'
      // The node may be tappable without having a more concrete role set on it,
      // such as "button". It will just have a tap handler. This could lead to
      // sized span to be chosen as the label representation strategy. However,
      // when pointer events land on the span the DOM `target` becomes the span
      // rather than the tappable element, and that breaks the debouncing logic
      // in `pointer_binding.dart`.
      ..pointerEvents = 'none';
    semanticsObject.element.appendChild(_domText);
  }

  final DomElement _domText = domDocument.createElement('span');
  String? _previousLabel;
  ui.Size? _previousSize;

  @override
  void update(String label) {
    final ui.Size? size = semanticsObject.rect?.size;
    final bool labelChanged = label != _previousLabel;
    final bool sizeChanged = size != _previousSize;

    // Label must be updated before sizing because the size depends on text
    // content.
    if (labelChanged) {
      _domText.text = label;
    }

    // This code makes the assumption that the DOM size of the element depends
    // solely on the text of the label. This is because text in the semantics
    // tree is unstyled. If this ever changes, this assumption will no longer
    // hold, and this code will need to be updated.
    if (labelChanged || sizeChanged) {
      _updateSize(size);
    }

    // Remember the last used data to shut off unnecessary updates.
    _previousLabel = label;
    _previousSize = size;
  }

  // Scales the text span (if any), such that the text matches the size of the
  // node. This is important because screen reader focus sizes itself tightly
  // around the text. Frequently, Flutter wants the focus to be different from
  // the text itself. For example, when you focus on a card widget containing a
  // piece of text, it is desirable that the focus covers the whole card, and
  // not just the text inside.
  //
  // The scaling may cause the text to become distorted, but that doesn't matter
  // because the semantic DOM is invisible.
  //
  // See: https://github.com/flutter/flutter/issues/146774
  void _updateSize(ui.Size? size) {
    if (size == null) {
      // There's no size to match => remove whatever stale sizing information was there.
      // Note, it is not necessary to always reset the transform before measuring,
      // as transform does not affect the offset size of the element. We do not
      // reset it unnecessarily to reduce the cost of setting properties
      // unnecessarily.
      _domText.style.transform = '';
      return;
    }

    if (_resizeQueue == null) {
      _resizeQueue = <_QueuedSizeUpdate>[];

      // Perform the adjustment in a post-update callback because the DOM layout
      // can only be performed when the elements are attached to the document,
      // but at this point the DOM tree is not yet finalized, and the element
      // corresponding to the semantic node may still be detached.
      semanticsObject.owner.addOneTimePostUpdateCallback(_updateSizes);
    }
    _resizeQueue!.add((representation: this, targetSize: size));
  }

  @override
  void cleanUp() {
    _domText.remove();
  }

  static List<_QueuedSizeUpdate>? _resizeQueue;

  static void _updateSizes() {
    final List<_QueuedSizeUpdate>? queue = _resizeQueue;

    // Eagerly reset the queue before doing any work. This ensures that if there
    // is an unexpected error while processing the queue, we don't end up in a
    // cycle that grows the queue indefinitely. Worst case, some text nodes end
    // up incorrectly sized, but that's a smaller problem compared to running
    // out of memory.
    _resizeQueue = null;

    assert(
      queue != null && queue.isNotEmpty,
      '_updateSizes was called with an empty _resizeQueue. This should never '
      'happend. If it does, please file an issue at '
      'https://github.com/flutter/flutter/issues/new/choose',
    );

    if (queue == null || queue.isEmpty) {
      // This should not happen, but if it does (e.g. something else fails and
      // caused the post-update callback to be called with an empty queue), do
      // not crash.
      return;
    }

    final List<_Measurement> measurements = <_Measurement>[];

    // Step 1: set `display` to `inline` so that the measurement measures the
    //         true size of the text. Update all spans in a batch so that the
    //         measurement can be done without changing CSS properties that
    //         trigger reflow.
    for (final _QueuedSizeUpdate update in queue) {
      update.representation._domText.style.display = 'inline';
    }

    // Step 2: measure all spans in a single batch prior to updating their CSS
    //         styles. This way, all measurements are taken with a single reflow.
    //         Interleaving measurements with updates, will cause the browser to
    //         reflow the page between measurements.
    for (final _QueuedSizeUpdate update in queue) {
      // Both clientWidth/Height and offsetWidth/Height provide a good
      // approximation for the purposes of sizing the focus ring of the text,
      // since there's no borders or scrollbars. The `offset` variant was chosen
      // mostly because it rounds the value to `int`, so the value is less
      // volatile and therefore would need fewer updates.
      //
      // getBoundingClientRect() was considered and rejected, because it provides
      // the rect in screen coordinates but this scale adjustment needs to be
      // local.
      final double domWidth = update.representation._domText.offsetWidth;
      final double domHeight = update.representation._domText.offsetHeight;
      measurements.add((
        representation: update.representation,
        domSize: ui.Size(domWidth, domHeight),
        targetSize: update.targetSize,
      ));
    }

    // Step 3: update all spans at a batch without taking any further DOM
    //         measurements, which avoids additional reflows.
    for (final _Measurement measurement in measurements) {
      final SizedSpanRepresentation representation = measurement.representation;
      final double domWidth = measurement.domSize.width;
      final double domHeight = measurement.domSize.height;
      final ui.Size targetSize = measurement.targetSize;

      // Reset back to `inline-block` (it was set to `inline` in Step 1).
      representation._domText.style.display = 'inline-block';

      if (domWidth < 1 && domHeight < 1) {
        // Don't bother dealing with divisions by tiny numbers. This probably means
        // the label is empty or doesn't contain anything that would be visible to
        // the user.
        representation._domText.style.transform = '';
      } else {
        final double scaleX = targetSize.width / domWidth;
        final double scaleY = targetSize.height / domHeight;
        representation._domText.style.transform = 'scale($scaleX, $scaleY)';
      }
    }

    assert(_resizeQueue == null, '_resizeQueue must be empty after it is processed.');
  }

  // The structure of the sized span label looks like this:
  //
  // <flt-semantics>
  //   <span>Here goes the label</span>
  // </flt-semantics>
  //
  // The target of the focus should be the <span>, not the <flt-semantics>.
  // Otherwise the browser will report the node as two separate nodes to the
  // screen reader. It would require the user to make an additional navigation
  // action to "step over" the <flt-semantics> to reach the <span> where the
  // text is. However, logically this DOM structure is just "one thing" as far
  // as the user is concerned, so both `tabindex` and the text of the label
  // should go on the same element.
  @override
  DomElement get focusTarget => _domText;
}

/// Renders the label for a [SemanticsObject] that can be scanned by screen
/// readers, web crawlers, and other automated agents.
///
/// See [computeDomSemanticsLabel] for the exact logic that constructs the label
/// of a semantic node.
class LabelAndValue extends SemanticBehavior {
  LabelAndValue(super.semanticsObject, super.owner, {required this.preferredRepresentation});

  /// The preferred representation of the label in the DOM.
  ///
  /// This value may be changed. Calling [update] after changing it will apply
  /// the new preference.
  ///
  /// If the node contains children, [LabelRepresentation.ariaLabel] is used
  /// instead.
  LabelRepresentation preferredRepresentation;

  @override
  void update() {
    final String? computedLabel = _computeLabel();

    if (computedLabel == null) {
      _cleanUpDom();
      return;
    }

    _getEffectiveRepresentation().update(computedLabel);
  }

  LabelRepresentationBehavior? _representation;

  /// Return the representation that should be used based on the current
  /// parameters of the semantic node.
  ///
  /// If the node has children always use an `aria-label`. Using extra child
  /// nodes to represent the label will cause layout shifts and confuse the
  /// screen reader. If the are no children, use the representation preferred
  /// by the role.
  LabelRepresentationBehavior _getEffectiveRepresentation() {
    final LabelRepresentation effectiveRepresentation =
        semanticsObject.hasChildren ? LabelRepresentation.ariaLabel : preferredRepresentation;

    LabelRepresentationBehavior? representation = _representation;
    if (representation == null || representation.kind != effectiveRepresentation) {
      representation?.cleanUp();
      _representation = representation = effectiveRepresentation.createBehavior(owner);
    }
    return representation;
  }

  /// Computes the final label to be assigned to the node.
  ///
  /// The label is a concatenation of tooltip, label, hint, and value, whichever
  /// combination is present.
  String? _computeLabel() {
    // If the node is incrementable the value is reported to the browser via
    // the respective role. We do not need to also render it again here.
    final bool shouldDisplayValue = !semanticsObject.isIncrementable && semanticsObject.hasValue;

    return computeDomSemanticsLabel(
      tooltip: semanticsObject.hasTooltip ? semanticsObject.tooltip : null,
      label: semanticsObject.hasLabel ? semanticsObject.label : null,
      hint: semanticsObject.hint,
      value: shouldDisplayValue ? semanticsObject.value : null,
    );
  }

  void _cleanUpDom() {
    _representation?.cleanUp();
  }

  @override
  void dispose() {
    super.dispose();
    _cleanUpDom();
  }

  /// Moves the focus to the element that carries the semantic label.
  ///
  /// Typically a node would be [Focusable] and focus request would be satisfied
  /// by transfering focus through the normal focusability features. However,
  /// sometimes accessibility focus needs to be moved to a non-focusable node,
  /// such as the title of a dialog. This method handles that situation.
  /// Different label representations use different DOM structures, so the
  /// actual work is delegated to [LabelRepresentationBehavior].
  void focusAsRouteDefault() {
    _getEffectiveRepresentation().focusAsRouteDefault();
  }
}

String? computeDomSemanticsLabel({String? tooltip, String? label, String? hint, String? value}) {
  final String? labelHintValue = _computeLabelHintValue(label: label, hint: hint, value: value);

  if (tooltip == null && labelHintValue == null) {
    return null;
  }

  final StringBuffer combinedValue = StringBuffer();
  if (tooltip != null) {
    combinedValue.write(tooltip);

    // Separate the tooltip from the rest via a line-break (if the rest exists).
    if (labelHintValue != null) {
      combinedValue.writeln();
    }
  }

  if (labelHintValue != null) {
    combinedValue.write(labelHintValue);
  }

  return combinedValue.isNotEmpty ? combinedValue.toString() : null;
}

String? _computeLabelHintValue({String? label, String? hint, String? value}) {
  final String combinedValue = <String?>[label, hint, value]
      .whereType<String>() // poor man's null filter
      .where((String element) => element.trim().isNotEmpty)
      .join(' ');
  return combinedValue.isNotEmpty ? combinedValue : null;
}
