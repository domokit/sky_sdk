// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../platform_dispatcher.dart';
import '../text_editing/text_editing.dart';
import 'semantics.dart';

/// Text editing used by accesibility mode.
///
/// [SemanticsTextEditingStrategy] assumes the caller will own the creation,
/// insertion and disposal of the DOM element. Due to this
/// [initializeElementPlacement], [initializeTextEditing] and
/// [disable] strategies are handled differently.
///
/// This class is still responsible for hooking up the DOM element with the
/// [HybridTextEditing] instance so that changes are communicated to Flutter.
class SemanticsTextEditingStrategy extends DefaultTextEditingStrategy {
  /// Creates a [SemanticsTextEditingStrategy] that eagerly instantiates
  /// [domElement] so the caller can insert it before calling
  /// [SemanticsTextEditingStrategy.enable].
  SemanticsTextEditingStrategy(super.owner);

  /// Initializes the [SemanticsTextEditingStrategy] singleton.
  ///
  /// This method must be called prior to accessing [instance].
  static SemanticsTextEditingStrategy ensureInitialized(HybridTextEditing owner) {
    if (_instance != null && _instance?.owner == owner) {
      return _instance!;
    }
    return _instance = SemanticsTextEditingStrategy(owner);
  }

  /// The [SemanticsTextEditingStrategy] singleton.
  static SemanticsTextEditingStrategy get instance => _instance!;
  static SemanticsTextEditingStrategy? _instance;

  /// The text field whose DOM element is currently used for editing.
  ///
  /// If this field is null, no editing takes place.
  SemanticTextField? activeTextField;

  /// Current input configuration supplied by the "flutter/textinput" channel.
  InputConfiguration? inputConfig;

  /// The semantics implementation does not operate on DOM nodes, but only
  /// remembers the config and callbacks. This is because the DOM nodes are
  /// supplied in the semantics update and enabled by [activate].
  @override
  void enable(
    InputConfiguration inputConfig, {
    required OnChangeCallback onChange,
    required OnActionCallback onAction,
  }) {
    this.inputConfig = inputConfig;
    this.onChange = onChange;
    this.onAction = onAction;
  }

  /// Attaches the DOM element owned by [textField] to the text editing
  /// strategy.
  ///
  /// This method must be called after [enable] to name sure that [inputConfig],
  /// [onChange], and [onAction] are not null.
  void activate(SemanticTextField textField) {
    assert(
      inputConfig != null && onChange != null && onAction != null,
      '"enable" should be called before "enableFromSemantics" and initialize input configuration',
    );

    if (activeTextField == textField) {
      // The specified field is already active. Skip.
      return;
    } else if (activeTextField != null) {
      // Another text field is currently active. Deactivate it before switching.
      disable();
    }

    activeTextField = textField;
    domElement = textField.editableElement;
    _syncStyle();
    super.enable(inputConfig!, onChange: onChange!, onAction: onAction!);
  }

  /// Detaches the DOM element owned by [textField] from this text editing
  /// strategy.
  ///
  /// Typically at this point the element loses focus (blurs) and stops being
  /// used for editing.
  void deactivate(SemanticTextField textField) {
    if (activeTextField == textField) {
      disable();
    }
  }

  @override
  void disable() {
    // We don't want to remove the DOM element because the caller is responsible
    // for that. However we still want to stop editing, cleanup the handlers.
    if (!isEnabled) {
      return;
    }

    isEnabled = false;
    style = null;
    geometry = null;

    for (int i = 0; i < subscriptions.length; i++) {
      subscriptions[i].cancel();
    }
    subscriptions.clear();
    lastEditingState = null;
    EnginePlatformDispatcher.instance.viewManager.safeBlur(activeDomElement);
    domElement = null;
    activeTextField = null;
    _queuedStyle = null;
  }

  @override
  void addEventHandlers() {
    if (inputConfiguration.autofillGroup != null) {
      subscriptions.addAll(inputConfiguration.autofillGroup!.addInputEventListeners());
    }

    // Subscribe to text and selection changes.
    subscriptions.add(DomSubscription(activeDomElement, 'input', handleChange));
    subscriptions.add(DomSubscription(activeDomElement, 'keydown', maybeSendAction));
    subscriptions.add(DomSubscription(domDocument, 'selectionchange', handleChange));
    preventDefaultForMouseEvents();
  }

  @override
  void initializeTextEditing(
    InputConfiguration inputConfig, {
    OnChangeCallback? onChange,
    OnActionCallback? onAction,
  }) {
    isEnabled = true;
    inputConfiguration = inputConfig;
    applyConfiguration(inputConfig);
  }

  @override
  void placeElement() {
    // If this text editing element is a part of an autofill group.
    if (hasAutofillGroup) {
      placeForm();
    }
    activeDomElement.focusWithoutScroll();
  }

  @override
  void initializeElementPlacement() {
    // Element placement is done by [SemanticTextField].
  }

  @override
  void placeForm() {}

  @override
  void updateElementPlacement(EditableTextGeometry textGeometry) {
    // Element placement is done by [SemanticTextField].
  }

  EditableTextStyle? _queuedStyle;

  @override
  void updateElementStyle(EditableTextStyle textStyle) {
    _queuedStyle = textStyle;
    _syncStyle();
  }

  /// Apply style to the element, if both style and element are available.
  ///
  /// Because style is supplied by the "flutter/textinput" channel and the DOM
  /// element is supplied by the semantics tree, the existence of both at the
  /// same time is not guaranteed.
  void _syncStyle() {
    if (_queuedStyle == null || domElement == null) {
      return;
    }
    super.updateElementStyle(_queuedStyle!);
  }
}

/// Manages semantics objects that represent editable text fields.
///
/// This role is implemented via a content-editable HTML element. This role does
/// not proactively switch modes depending on the current
/// [EngineSemanticsOwner.gestureMode]. However, in Chrome on Android it ignores
/// browser gestures when in pointer mode. In Safari on iOS pointer events are
/// used to detect text box invocation. This is because Safari issues touch
/// events even when VoiceOver is enabled.
class SemanticTextField extends SemanticRole {
  SemanticTextField(SemanticsObject semanticsObject)
    : super.blank(SemanticRoleKind.textField, semanticsObject) {
    _initializeEditableElement();
  }

  @override
  bool get acceptsPointerEvents => true;

  /// The element used for editing, e.g. `<input>`, `<textarea>`, which is
  /// different from the host [element].
  late final DomHTMLElement editableElement;

  @override
  bool focusAsRouteDefault() {
    editableElement.focusWithoutScroll();
    return true;
  }

  DomHTMLInputElement _createSingleLineField() {
    return createDomHTMLInputElement()
      ..type = semanticsObject.hasFlag(ui.SemanticsFlag.isObscured) ? 'password' : 'text';
  }

  DomHTMLTextAreaElement _createMultiLineField() {
    final textArea = createDomHTMLTextAreaElement();

    if (semanticsObject.hasFlag(ui.SemanticsFlag.isObscured)) {
      // -webkit-text-security is not standard, but it's the best we can do.
      // Another option would be to create a single-line <input type="password">
      // but that may have layout quirks, since it cannot represent multi-line
      // text. Worst case with -webkit-text-security is the browser does not
      // support it and it does not obscure text. However, that's not a huge
      // problem because semantic DOM is already invisible.
      textArea.style.setProperty('-webkit-text-security', 'circle');
    }

    return textArea;
  }

  void _initializeEditableElement() {
    editableElement =
        semanticsObject.hasFlag(ui.SemanticsFlag.isMultiline)
            ? _createMultiLineField()
            : _createSingleLineField();
    _updateEnabledState();

    // On iOS, even though the semantic text field is transparent, the cursor
    // and text highlighting are still visible. The cursor and text selection
    // are made invisible by CSS in [StyleManager.attachGlobalStyles].
    // But there's one more case where iOS highlights text. That's when there's
    // and autocorrect suggestion. To disable that, we have to do the following:
    editableElement
      ..spellcheck = false
      ..setAttribute('autocorrect', 'off')
      ..setAttribute('autocomplete', 'off')
      ..setAttribute('data-semantics-role', 'text-field');

    editableElement.style
      ..position = 'absolute'
      // `top` and `left` are intentionally set to zero here.
      //
      // The text field would live inside a `<flt-semantics>` which should
      // already be positioned using semantics.rect.
      //
      // See also:
      //
      // * [SemanticsObject.recomputePositionAndSize], which sets the position
      //   and size of the parent `<flt-semantics>` element.
      ..top = '0'
      ..left = '0'
      ..width = '${semanticsObject.rect!.width}px'
      ..height = '${semanticsObject.rect!.height}px';
    append(editableElement);

    editableElement.addEventListener(
      'focus',
      createDomEventListener((DomEvent event) {
        // IMPORTANT: because this event listener can be triggered by either or
        // both a "focus" and a "click" DOM events, this code must be idempotent.
        EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
          viewId,
          semanticsObject.id,
          ui.SemanticsAction.focus,
          null,
        );
      }),
    );
    editableElement.addEventListener(
      'click',
      createDomEventListener((DomEvent event) {
        editableElement.focusWithoutScroll();
      }),
    );
    editableElement.addEventListener(
      'blur',
      createDomEventListener((DomEvent event) {
        SemanticsTextEditingStrategy._instance?.deactivate(this);
      }),
    );
  }

  @override
  void update() {
    super.update();

    _updateEnabledState();
    editableElement.style
      ..width = '${semanticsObject.rect!.width}px'
      ..height = '${semanticsObject.rect!.height}px';

    if (semanticsObject.hasFocus) {
      if (domDocument.activeElement != editableElement && semanticsObject.isEnabled) {
        semanticsObject.owner.addOneTimePostUpdateCallback(() {
          editableElement.focusWithoutScroll();
        });
      }
      SemanticsTextEditingStrategy._instance?.activate(this);
    }

    if (semanticsObject.hasLabel) {
      if (semanticsObject.isLabelDirty) {
        editableElement.setAttribute('aria-label', semanticsObject.label!);
      }
    } else {
      editableElement.removeAttribute('aria-label');
    }
  }

  void _updateEnabledState() {
    (editableElement as DomElementWithDisabledProperty).disabled = !semanticsObject.isEnabled;
  }

  @override
  void dispose() {
    super.dispose();
    SemanticsTextEditingStrategy._instance?.deactivate(this);
  }
}
