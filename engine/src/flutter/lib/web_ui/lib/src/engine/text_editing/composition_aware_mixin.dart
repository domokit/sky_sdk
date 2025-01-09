// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import '../dom.dart';
import 'text_editing.dart';

/// Provides default functionality for listening to HTML composition events.
///
/// A class with this mixin generally calls [determineCompositionState] in order to update
/// an [EditingState] with new composition values; namely, [EditingState.composingBaseOffset]
/// and [EditingState.composingExtentOffset].
///
/// A class with this mixin should call [addCompositionEventHandlers] on initalization, and
/// [removeCompositionEventHandlers] on deinitalization.
///
/// See also:
///
/// * [EditingState], the state of a text field that [CompositionAwareMixin] updates.
/// * [DefaultTextEditingStrategy], the primary implementer of [CompositionAwareMixin].
mixin CompositionAwareMixin {
  /// The name of the HTML composition event type that triggers on starting a composition.
  static const String _kCompositionStart = 'compositionstart';

  /// The name of the browser composition event type that triggers on updating a composition.
  static const String _kCompositionUpdate = 'compositionupdate';

  /// The name of the browser composition event type that triggers on ending a composition.
  static const String _kCompositionEnd = 'compositionend';

  late final DomEventListener _compositionStartListener = createDomEventListener(
    _handleCompositionStart,
  );
  late final DomEventListener _compositionUpdateListener = createDomEventListener(
    _handleCompositionUpdate,
  );
  late final DomEventListener _compositionEndListener = createDomEventListener(
    _handleCompositionEnd,
  );

  /// The currently composing text in the `domElement`.
  ///
  /// Will be null if composing just started, ended, or no composing is being done.
  /// This member is kept up to date provided compositionEventHandlers are in place,
  /// so it is safe to reference it to get the current composingText.
  String? composingText;

  void addCompositionEventHandlers(DomHTMLElement domElement) {
    domElement.addEventListener(_kCompositionStart, _compositionStartListener);
    domElement.addEventListener(_kCompositionUpdate, _compositionUpdateListener);
    domElement.addEventListener(_kCompositionEnd, _compositionEndListener);
  }

  void removeCompositionEventHandlers(DomHTMLElement domElement) {
    domElement.removeEventListener(_kCompositionStart, _compositionStartListener);
    domElement.removeEventListener(_kCompositionUpdate, _compositionUpdateListener);
    domElement.removeEventListener(_kCompositionEnd, _compositionEndListener);
  }

  JSVoid _handleCompositionStart(DomEvent event) {
    composingText = null;
  }

  JSVoid _handleCompositionUpdate(DomEvent event) {
    if (domInstanceOfString(event, 'CompositionEvent')) {
      composingText = (event as DomCompositionEvent).data;
    }
  }

  JSVoid _handleCompositionEnd(DomEvent event) {
    composingText = null;
  }

  EditingState determineCompositionState(EditingState editingState) {
    if (editingState.extentOffset == null || composingText == null || editingState.text == null) {
      return editingState;
    }

    final int composingBase = editingState.extentOffset! - composingText!.length;

    if (composingBase < 0) {
      return editingState;
    }

    return editingState.copyWith(
      composingBaseOffset: composingBase,
      composingExtentOffset: composingBase + composingText!.length,
    );
  }
}
