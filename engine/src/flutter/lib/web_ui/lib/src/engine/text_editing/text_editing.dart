// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import '../mouse/prevent_default.dart';
import '../platform_dispatcher.dart';
import '../safe_browser_api.dart';
import '../semantics.dart';
import '../services.dart';
import '../text/paragraph.dart';
import '../util.dart';
import '../view_embedder/flutter_view_manager.dart';
import '../window.dart';
import 'autofill_hint.dart';
import 'composition_aware_mixin.dart';
import 'input_action.dart';
import 'input_type.dart';
import 'text_capitalization.dart';

/// Make the content editable span visible to facilitate debugging.
bool _debugVisibleTextEditing = false;

/// Set this to `true` to print when text input commands are scheduled and run.
bool _debugPrintTextInputCommands = false;

/// The `keyCode` of the "Enter" key.
const int _kReturnKeyCode = 13;

/// Offset in pixels to place an element outside of the screen.
const int offScreenOffset = -9999;

/// Blink and Webkit engines, bring an overlay on top of the text field when it
/// is autofilled.
bool browserHasAutofillOverlay() =>
    ui_web.browser.browserEngine == ui_web.BrowserEngine.blink ||
    ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit;

/// `transparentTextEditing` class is configured to make the autofill overlay
/// transparent.
const String transparentTextEditingClass = 'transparentTextEditing';

void _emptyCallback(dynamic _) {}

/// These style attributes are constant throughout the life time of an input
/// element.
///
/// They are assigned once during the creation of the DOM element.
void _setStaticStyleAttributes(DomHTMLElement domElement) {
  domElement.classList.add(HybridTextEditing.textEditingClass);

  final DomCSSStyleDeclaration elementStyle = domElement.style;
  elementStyle
    // Prevent (forced-colors: active) from making our invisible text fields visible.
    // For more details, see: https://developer.mozilla.org/en-US/docs/Web/CSS/forced-color-adjust
    ..setProperty('forced-color-adjust', 'none')
    ..whiteSpace = 'pre-wrap'
    ..alignContent = 'center'
    ..position = 'absolute'
    ..top = '0'
    ..left = '0'
    ..padding = '0'
    ..opacity = '1'
    ..color = 'transparent'
    ..backgroundColor = 'transparent'
    ..background = 'transparent'
    // This property makes the input's blinking cursor transparent.
    ..caretColor = 'transparent'
    ..outline = 'none'
    ..border = 'none'
    ..resize = 'none'
    ..textShadow = 'none'
    ..overflow = 'hidden'
    ..transformOrigin = '0 0 0';

  if (browserHasAutofillOverlay()) {
    domElement.classList.add(transparentTextEditingClass);
  }

  if (_debugVisibleTextEditing) {
    elementStyle
      ..color = 'purple'
      ..outline = '1px solid purple';
  }
}

/// Sets attributes to hide autofill elements.
///
/// These style attributes are constant throughout the life time of an input
/// element.
///
/// They are assigned once during the creation of the DOM element.
void _styleAutofillElements(
  DomHTMLElement domElement, {
  bool isOffScreen = false,
  bool shouldHideElement = true,
  bool shouldDisablePointerEvents = false,
}) {
  final DomCSSStyleDeclaration elementStyle = domElement.style;
  elementStyle
    ..whiteSpace = 'pre-wrap'
    ..alignContent = 'center'
    ..padding = '0'
    ..opacity = '1'
    ..color = 'transparent'
    ..backgroundColor = 'transparent'
    ..background = 'transparent'
    ..outline = 'none'
    ..border = 'none'
    ..resize = 'none'
    ..textShadow = 'transparent'
    ..transformOrigin = '0 0 0';

  if (isOffScreen) {
    elementStyle
      ..top = '${offScreenOffset}px'
      ..left = '${offScreenOffset}px';
  }

  if (shouldHideElement) {
    elementStyle
      ..width = '0'
      ..height = '0';
  }

  if (shouldDisablePointerEvents) {
    elementStyle.pointerEvents = 'none';
  }

  if (browserHasAutofillOverlay()) {
    domElement.classList.add(transparentTextEditingClass);
  }

  /// This property makes the input's blinking cursor transparent.
  elementStyle.setProperty('caret-color', 'transparent');
}

void _ensureEditingElementInView(DomElement element, int viewId) {
  final bool isAlreadyAppended = element.isConnected ?? false;
  if (!isAlreadyAppended) {
    // If the element is not already appended to a view, we don't need to move
    // it anywhere.
    return;
  }

  final FlutterViewManager viewManager = EnginePlatformDispatcher.instance.viewManager;
  final EngineFlutterView? currentView = viewManager.findViewForElement(element);
  if (currentView == null) {
    // For some reason, the input element was in the DOM, but it wasn't part of
    // any Flutter view. Should we throw?
    return;
  }

  if (currentView.viewId != viewId) {
    _insertEditingElementInView(element, viewId);
  }
}

void _insertEditingElementInView(DomElement element, int viewId) {
  final FlutterViewManager viewManager = EnginePlatformDispatcher.instance.viewManager;
  final EngineFlutterView? view = viewManager[viewId];
  assert(
    view != null,
    'Could not find View with id $viewId. This should never happen, please file a bug!',
  );
  view!.dom.textEditingHost.append(element);
}

/// Form that contains all the fields in the same AutofillGroup.
///
/// An [EngineAutofillForm] will only be constructed when autofill is enabled
/// (the default) on the current input field. See the [fromFrameworkMessage]
/// static method.
class EngineAutofillForm {
  EngineAutofillForm({
    required this.viewId,
    required this.formElement,
    this.elements,
    this.items,
    this.formIdentifier = '',
    this.insertionReferenceNode,
  });

  final DomHTMLFormElement formElement;

  final Map<String, DomHTMLElement>? elements;

  final Map<String, AutofillInfo>? items;

  final DomHTMLElement? insertionReferenceNode;

  /// Identifier for the form.
  ///
  /// It is constructed by concatenating unique ids of input elements on the
  /// form.
  ///
  /// It is used for storing the form until submission.
  /// See [formsOnTheDom].
  final String formIdentifier;

  /// The ID of the view that this form is rendered into.
  final int viewId;

  /// Creates an [EngineAutofillFrom] from the JSON representation of a Flutter
  /// framework `TextInputConfiguration` object.
  ///
  /// The `focusedElementAutofill` argument corresponds to the "autofill" field
  /// in a `TextInputConfiguration`. Not having this field indicates autofill
  /// is explicitly disabled on the text field by the developer.
  ///
  /// The `fields` argument corresponds to the "fields" field in a
  /// `TextInputConfiguration`.
  ///
  /// Returns null if autofill is disabled for the input field.
  static EngineAutofillForm? fromFrameworkMessage(
    int viewId,
    Map<String, dynamic>? focusedElementAutofill,
    List<dynamic>? fields,
  ) {
    // Autofill value will be null if the developer explicitly disables it on
    // the input field.
    if (focusedElementAutofill == null) {
      return null;
    }

    // If there is only one text field in the autofill model, `fields` will be
    // null. `focusedElementAutofill` contains the information about the one
    // text field.
    final Map<String, DomHTMLElement> elements = <String, DomHTMLElement>{};
    final Map<String, AutofillInfo> items = <String, AutofillInfo>{};
    final DomHTMLFormElement formElement = createDomHTMLFormElement();
    final bool isSafariDesktopStrategy = textEditing.strategy is SafariDesktopTextEditingStrategy;
    DomHTMLElement? insertionReferenceNode;

    // Validation is in the framework side.
    formElement.noValidate = true;
    formElement.method = 'post';
    formElement.action = '#';
    formElement.addEventListener('submit', preventDefaultListener);

    // We need to explicitly disable pointer events on the form in Safari Desktop,
    // so that we don't have pointer event collisions if users hover over or click
    // into the invisible autofill elements within the form.
    _styleAutofillElements(formElement, shouldDisablePointerEvents: isSafariDesktopStrategy);

    // We keep the ids in a list then sort them later, in case the text fields'
    // locations are re-ordered on the framework side.
    final List<String> ids = List<String>.empty(growable: true);

    // The focused text editing element will not be created here.
    final AutofillInfo focusedElement = AutofillInfo.fromFrameworkMessage(focusedElementAutofill);

    if (fields != null) {
      bool fieldIsFocusedElement = false;
      for (final Map<String, dynamic> field in fields.cast<Map<String, dynamic>>()) {
        final Map<String, dynamic> autofillInfo = field.readJson('autofill');
        final AutofillInfo autofill = AutofillInfo.fromFrameworkMessage(
          autofillInfo,
          textCapitalization: TextCapitalizationConfig.fromInputConfiguration(
            field.readString('textCapitalization'),
          ),
        );

        ids.add(autofill.uniqueIdentifier);

        if (autofill.uniqueIdentifier != focusedElement.uniqueIdentifier) {
          final EngineInputType engineInputType = EngineInputType.fromName(
            field.readJson('inputType').readString('name'),
          );

          final DomHTMLElement htmlElement = engineInputType.createDomElement();
          autofill.editingState.applyToDomElement(htmlElement);
          autofill.applyToDomElement(htmlElement);

          // Safari Desktop does not respect elements that are invisible (or
          // have no size) and that leads to issues with autofill only partially
          // working (ref: https://github.com/flutter/flutter/issues/71275).
          // Thus, we have to make sure that the elements remain invisible to users,
          // but not to Safari for autofill to work. Since these elements are
          // sized and placed on the DOM, we also have to disable pointer events.
          _styleAutofillElements(
            htmlElement,
            shouldHideElement: !isSafariDesktopStrategy,
            shouldDisablePointerEvents: isSafariDesktopStrategy,
          );

          items[autofill.uniqueIdentifier] = autofill;
          elements[autofill.uniqueIdentifier] = htmlElement;
          formElement.append(htmlElement);

          // We want to track the node in the position directly after our focused
          // element, so we can later insert that element in the correct position
          // right before this node.
          if (fieldIsFocusedElement) {
            insertionReferenceNode = htmlElement;
            fieldIsFocusedElement = false;
          }
        } else {
          // current field is the focused element that we create elsewhere
          fieldIsFocusedElement = true;
        }
      }
    } else {
      // There is one input element in the form.
      ids.add(focusedElement.uniqueIdentifier);
    }

    ids.sort();
    final StringBuffer idBuffer = StringBuffer();

    // Add a separator between element identifiers.
    for (final String id in ids) {
      if (idBuffer.length > 0) {
        idBuffer.write('*');
      }
      idBuffer.write(id);
    }

    final String formIdentifier = idBuffer.toString();

    // If a form with the same Autofill elements is already on the dom, remove
    // it from DOM.
    final DomHTMLFormElement? form = formsOnTheDom[formIdentifier];
    form?.remove();

    // In order to submit the form when Framework sends a `TextInput.commit`
    // message, we add a submit button to the form.
    // The -1 tab index value makes this element not reachable by keyboard.
    final DomHTMLInputElement submitButton = createDomHTMLInputElement()..tabIndex = -1;
    _styleAutofillElements(submitButton, isOffScreen: true);
    submitButton.className = 'submitBtn';
    submitButton.type = 'submit';

    formElement.append(submitButton);

    // If the focused node is at the end of the form, we'll default to inserting
    // it before the submit field.
    insertionReferenceNode ??= submitButton;

    return EngineAutofillForm(
      viewId: viewId,
      formElement: formElement,
      elements: elements,
      items: items,
      formIdentifier: formIdentifier,
      insertionReferenceNode: insertionReferenceNode,
    );
  }

  void placeForm(DomHTMLElement mainTextEditingElement) {
    // Since we're disabling pointer events on the form to fix Safari autofill,
    // we need to explicitly set pointer events on the active input element in
    // order to calculate the correct pointer event offsets.
    // See: https://github.com/flutter/flutter/issues/136006
    if (textEditing.strategy is SafariDesktopTextEditingStrategy) {
      mainTextEditingElement.style.pointerEvents = 'all';
    }

    formElement.insertBefore(mainTextEditingElement, insertionReferenceNode);
    _insertEditingElementInView(formElement, viewId);
  }

  void storeForm() {
    formsOnTheDom[formIdentifier] = formElement;
    _styleAutofillElements(formElement, isOffScreen: true);
  }

  /// Listens to `onInput` event on the form fields.
  ///
  /// Registering to the listeners could have been done in the constructor.
  /// On the other hand, overall for text editing there is already a lifecycle
  /// for subscriptions: All the subscriptions of the DOM elements are to the
  /// `subscriptions` property of [DefaultTextEditingStrategy].
  /// [TextEditingStrategy] manages all subscription lifecyle. All
  /// listeners with no exceptions are added during
  /// [TextEditingStrategy.addEventHandlers] method call and all
  /// listeners are removed during [TextEditingStrategy.disable] method call.
  List<DomSubscription> addInputEventListeners() {
    final Iterable<String> keys = elements!.keys;
    final List<DomSubscription> subscriptions = <DomSubscription>[];

    void addSubscriptionForKey(String key) {
      final DomElement element = elements![key]!;
      subscriptions.add(
        DomSubscription(element, 'input', (DomEvent e) {
          if (items![key] == null) {
            throw StateError('AutofillInfo must have a valid uniqueIdentifier.');
          } else {
            final AutofillInfo autofillInfo = items![key]!;
            handleChange(element, autofillInfo);
          }
        }),
      );
    }

    keys.forEach(addSubscriptionForKey);
    return subscriptions;
  }

  void handleChange(DomElement domElement, AutofillInfo autofillInfo) {
    final EditingState newEditingState = EditingState.fromDomElement(domElement as DomHTMLElement);

    _sendAutofillEditingState(autofillInfo.uniqueIdentifier, newEditingState);
  }

  /// Sends the 'TextInputClient.updateEditingStateWithTag' message to the framework.
  void _sendAutofillEditingState(String? tag, EditingState editingState) {
    EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
      'flutter/textinput',
      const JSONMethodCodec().encodeMethodCall(
        MethodCall('TextInputClient.updateEditingStateWithTag', <dynamic>[
          0,
          <String?, dynamic>{tag: editingState.toFlutter()},
        ]),
      ),
      _emptyCallback,
    );
  }
}

/// Autofill related values.
///
/// These values are to be used when a text field have autofill enabled.
@visibleForTesting
class AutofillInfo {
  AutofillInfo({
    required this.editingState,
    required this.uniqueIdentifier,
    required this.autofillHint,
    required this.textCapitalization,
    this.placeholder,
  });

  factory AutofillInfo.fromFrameworkMessage(
    Map<String, dynamic> autofill, {
    TextCapitalizationConfig textCapitalization =
        const TextCapitalizationConfig.defaultCapitalization(),
  }) {
    final String uniqueIdentifier = autofill.readString('uniqueIdentifier');
    final List<dynamic>? hintsList = autofill.tryList('hints');
    final String? firstHint =
        (hintsList == null || hintsList.isEmpty) ? null : hintsList.first as String;
    final EditingState editingState = EditingState.fromFrameworkMessage(
      autofill.readJson('editingValue'),
    );
    return AutofillInfo(
      uniqueIdentifier: uniqueIdentifier,
      autofillHint:
          (firstHint != null) ? BrowserAutofillHints.instance.flutterToEngine(firstHint) : null,
      editingState: editingState,
      placeholder: autofill.tryString('hintText'),
      textCapitalization: textCapitalization,
    );
  }

  /// The current text and selection state of a text field.
  final EditingState editingState;

  /// Unique value set by the developer or generated by the framework.
  ///
  /// Used as id of the text field.
  ///
  /// An example an id generated by the framework: `EditableText-285283643`.
  final String uniqueIdentifier;

  /// Information on how should autofilled text capitalized.
  ///
  /// For example for [TextCapitalization.characters] each letter is converted
  /// to upper case.
  ///
  /// This value is not necessary for autofilling the focused element since
  /// [DefaultTextEditingStrategy.inputConfiguration] already has this
  /// information.
  ///
  /// On the other hand for the multi element forms, for the input elements
  /// other the focused field, we need to use this information.
  final TextCapitalizationConfig textCapitalization;

  /// The type of information expected in the field, specified by the developer.
  ///
  /// Used as a guidance to the browser as to the type of information expected
  /// in the field.
  /// See: https://developer.mozilla.org/en-US/docs/Web/HTML/Attributes/autocomplete
  final String? autofillHint;

  /// The optional hint text placed on the view that typically suggests what
  /// sort of input the field accepts, for example "enter your password here".
  ///
  /// If the developer does not specify any [autofillHints], the [placeholder]
  /// can be a useful indication to the platform autofill service as to what
  /// information is expected in this field.
  final String? placeholder;

  void applyToDomElement(DomHTMLElement domElement, {bool focusedElement = false}) {
    final String? autofillHint = this.autofillHint;
    final String? placeholder = this.placeholder;
    if (domInstanceOfString(domElement, 'HTMLInputElement')) {
      final DomHTMLInputElement element = domElement as DomHTMLInputElement;
      if (placeholder != null) {
        element.placeholder = placeholder;
      }
      if (autofillHint != null) {
        element.name = autofillHint;
        element.id = autofillHint;
        if (autofillHint.contains('password')) {
          element.type = 'password';
        } else {
          element.type = 'text';
        }
      }
      element.autocomplete = autofillHint ?? 'on';
    } else if (domInstanceOfString(domElement, 'HTMLTextAreaElement')) {
      final DomHTMLTextAreaElement element = domElement as DomHTMLTextAreaElement;
      if (placeholder != null) {
        element.placeholder = placeholder;
      }
      if (autofillHint != null) {
        element.name = autofillHint;
        element.id = autofillHint;
      }
      element.setAttribute('autocomplete', autofillHint ?? 'on');
    }
  }
}

/// Replaces a range of text in the original string with the text given in the
/// replacement string.
String _replace(String originalText, String replacementText, ui.TextRange replacedRange) {
  assert(replacedRange.isValid);
  assert(replacedRange.start <= originalText.length && replacedRange.end <= originalText.length);

  final ui.TextRange normalizedRange = ui.TextRange(
    start: math.min(replacedRange.start, replacedRange.end),
    end: math.max(replacedRange.start, replacedRange.end),
  );

  return normalizedRange.textBefore(originalText) +
      replacementText +
      normalizedRange.textAfter(originalText);
}

/// The change between the last editing state and the current editing state
/// of a text field.
///
/// This is packaged into a JSON and sent to the framework
/// to be processed into a concrete [TextEditingDelta].
class TextEditingDeltaState {
  TextEditingDeltaState({
    this.oldText = '',
    this.deltaText = '',
    this.deltaStart = -1,
    this.deltaEnd = -1,
    this.baseOffset,
    this.extentOffset,
    this.composingOffset,
    this.composingExtent,
  });

  /// Infers the correct delta values based on information from the new editing state
  /// and the last editing state.
  ///
  /// For a deletion, the length and the direction of the deletion (backward or forward)
  /// are calculated by comparing the new and last editing states.
  /// If the deletion is backward, the length is susbtracted from the [deltaEnd]
  /// that we set when beforeinput was fired to determine the [deltaStart].
  /// If the deletion is forward, [deltaStart] is set to the new editing state baseOffset
  /// and [deltaEnd] is set to [deltaStart] incremented by the length of the deletion.
  ///
  /// For a replacement at a selection we set the [deltaStart] to be the beginning of the selection
  /// from the last editing state.
  ///
  /// For the composing region we check if a composing range was captured by the compositionupdate event,
  /// we have a non empty [deltaText], and that we did not have an active selection. An active selection
  /// would mean we are not composing.
  ///
  /// We then verify that the delta we collected results in the text contained within the new editing state
  /// when applied to the last editing state. If it is not then we use our new editing state as the source of truth,
  /// and use regex to find the correct [deltaStart] and [deltaEnd].
  static TextEditingDeltaState inferDeltaState(
    EditingState newEditingState,
    EditingState? lastEditingState,
    TextEditingDeltaState lastTextEditingDeltaState,
  ) {
    final TextEditingDeltaState newTextEditingDeltaState = lastTextEditingDeltaState.copyWith();
    final bool previousSelectionWasCollapsed =
        lastEditingState?.baseOffset == lastEditingState?.extentOffset;
    final bool isTextBeingRemoved =
        newTextEditingDeltaState.deltaText.isEmpty && newTextEditingDeltaState.deltaEnd != -1;
    final bool isTextBeingChangedAtActiveSelection =
        newTextEditingDeltaState.deltaText.isNotEmpty && !previousSelectionWasCollapsed;

    if (isTextBeingRemoved) {
      // When text is deleted outside of the composing region or is cut using the native toolbar,
      // we calculate the length of the deleted text by comparing the new and old editing state lengths.
      // If the deletion is backward, the length is subtracted from the [deltaEnd]
      // that we set when beforeinput was fired to determine the [deltaStart].
      // If the deletion is forward, [deltaStart] is set to the new editing state baseOffset
      // and [deltaEnd] is set to [deltaStart] incremented by the length of the deletion.
      final int deletedLength =
          newTextEditingDeltaState.oldText.length - newEditingState.text!.length;
      final bool backwardDeletion = newEditingState.baseOffset != lastEditingState?.baseOffset;
      if (backwardDeletion) {
        newTextEditingDeltaState.deltaStart = newTextEditingDeltaState.deltaEnd - deletedLength;
      } else {
        // Forward deletion
        newTextEditingDeltaState.deltaStart = newEditingState.baseOffset!;
        newTextEditingDeltaState.deltaEnd = newTextEditingDeltaState.deltaStart + deletedLength;
      }
    } else if (isTextBeingChangedAtActiveSelection) {
      final bool isPreviousSelectionInverted =
          lastEditingState!.baseOffset! > lastEditingState.extentOffset!;
      // When a selection of text is replaced by a copy/paste operation we set the starting range
      // of the delta to be the beginning of the selection of the previous editing state.
      newTextEditingDeltaState.deltaStart =
          isPreviousSelectionInverted
              ? lastEditingState.extentOffset!
              : lastEditingState.baseOffset!;
    }

    // If we are composing then set the delta range to the composing region we
    // captured in compositionupdate.
    final bool isCurrentlyComposing =
        newTextEditingDeltaState.composingOffset != null &&
        newTextEditingDeltaState.composingOffset != newTextEditingDeltaState.composingExtent;
    if (newTextEditingDeltaState.deltaText.isNotEmpty &&
        previousSelectionWasCollapsed &&
        isCurrentlyComposing) {
      newTextEditingDeltaState.deltaStart = newTextEditingDeltaState.composingOffset!;
    }

    final bool isDeltaRangeEmpty =
        newTextEditingDeltaState.deltaStart == -1 &&
        newTextEditingDeltaState.deltaStart == newTextEditingDeltaState.deltaEnd;
    if (!isDeltaRangeEmpty) {
      // To verify the range of our delta we should compare the newEditingState's
      // text with the delta applied to the oldText. If they differ then capture
      // the correct delta range from the newEditingState's text value.
      //
      // We can assume the deltaText for additions and replacements to the text value
      // are accurate. What may not be accurate is the range of the delta.
      //
      // We can think of the newEditingState as our source of truth.
      //
      // This verification is needed for cases such as the insertion of a period
      // after a double space, and the insertion of an accented character through
      // a native composing menu.
      final ui.TextRange replacementRange = ui.TextRange(
        start: newTextEditingDeltaState.deltaStart,
        end: newTextEditingDeltaState.deltaEnd,
      );
      final String textAfterDelta = _replace(
        newTextEditingDeltaState.oldText,
        newTextEditingDeltaState.deltaText,
        replacementRange,
      );
      final bool isDeltaVerified = textAfterDelta == newEditingState.text!;

      if (!isDeltaVerified) {
        // 1. Find all matches for deltaText.
        // 2. Apply matches/replacement to oldText until oldText matches the
        // new editing state's text value.
        final bool isPeriodInsertion = newTextEditingDeltaState.deltaText.contains('.');
        final RegExp deltaTextPattern = RegExp(RegExp.escape(newTextEditingDeltaState.deltaText));
        for (final Match match in deltaTextPattern.allMatches(newEditingState.text!)) {
          String textAfterMatch;
          int actualEnd;
          final bool isMatchWithinOldTextBounds =
              match.start >= 0 && match.end <= newTextEditingDeltaState.oldText.length;
          if (!isMatchWithinOldTextBounds) {
            actualEnd = match.start + newTextEditingDeltaState.deltaText.length - 1;
            textAfterMatch = _replace(
              newTextEditingDeltaState.oldText,
              newTextEditingDeltaState.deltaText,
              ui.TextRange(start: match.start, end: actualEnd),
            );
          } else {
            actualEnd = actualEnd = isPeriodInsertion ? match.end - 1 : match.end;
            textAfterMatch = _replace(
              newTextEditingDeltaState.oldText,
              newTextEditingDeltaState.deltaText,
              ui.TextRange(start: match.start, end: actualEnd),
            );
          }

          if (textAfterMatch == newEditingState.text!) {
            newTextEditingDeltaState.deltaStart = match.start;
            newTextEditingDeltaState.deltaEnd = actualEnd;
            break;
          }
        }
      }
    }

    // Update selection of the delta using information from the new editing state.
    newTextEditingDeltaState.baseOffset = newEditingState.baseOffset;
    newTextEditingDeltaState.extentOffset = newEditingState.extentOffset;

    return newTextEditingDeltaState;
  }

  /// The text before the text field was updated.
  String oldText;

  /// The text that is being inserted/replaced into the text field.
  /// This will be an empty string for deletions and non text updates
  /// such as selection updates.
  String deltaText;

  /// The position in the text field where the change begins.
  ///
  /// Has a default value of -1 to signify an empty range.
  int deltaStart;

  /// The position in the text field where the change ends.
  ///
  /// Has a default value of -1 to signify an empty range.
  int deltaEnd;

  /// The updated starting position of the selection in the text field.
  int? baseOffset;

  /// The updated terminating position of the selection in the text field.
  int? extentOffset;

  /// The starting position of the composing region.
  int? composingOffset;

  /// The terminating position of the composing region.
  int? composingExtent;

  Map<String, dynamic> toFlutter() => <String, dynamic>{
    'deltas': <Map<String, dynamic>>[
      <String, dynamic>{
        'oldText': oldText,
        'deltaText': deltaText,
        'deltaStart': deltaStart,
        'deltaEnd': deltaEnd,
        'selectionBase': baseOffset,
        'selectionExtent': extentOffset,
        'composingBase': composingOffset,
        'composingExtent': composingExtent,
      },
    ],
  };

  TextEditingDeltaState copyWith({
    String? oldText,
    String? deltaText,
    int? deltaStart,
    int? deltaEnd,
    int? baseOffset,
    int? extentOffset,
    int? composingOffset,
    int? composingExtent,
  }) {
    return TextEditingDeltaState(
      oldText: oldText ?? this.oldText,
      deltaText: deltaText ?? this.deltaText,
      deltaStart: deltaStart ?? this.deltaStart,
      deltaEnd: deltaEnd ?? this.deltaEnd,
      baseOffset: baseOffset ?? this.baseOffset,
      extentOffset: extentOffset ?? this.extentOffset,
      composingOffset: composingOffset ?? this.composingOffset,
      composingExtent: composingExtent ?? this.composingExtent,
    );
  }
}

/// The current text and selection state of a text field.
class EditingState {
  EditingState({
    this.text,
    int? baseOffset,
    int? extentOffset,
    this.composingBaseOffset = -1,
    this.composingExtentOffset = -1,
  }) : // Don't allow negative numbers.
       baseOffset = math.max(0, baseOffset ?? 0),
       // Don't allow negative numbers.
       extentOffset = math.max(0, extentOffset ?? 0);

  /// Creates an [EditingState] instance using values from an editing state Map
  /// coming from Flutter.
  ///
  /// The `editingState` Map has the following structure:
  /// ```json
  /// {
  ///   "text": "The text here",
  ///   "selectionBase": 0,
  ///   "selectionExtent": 0,
  ///   "selectionAffinity": "TextAffinity.upstream",
  ///   "selectionIsDirectional": false,
  ///   "composingBase": -1,
  ///   "composingExtent": -1
  /// }
  /// ```
  ///
  /// Flutter Framework can send the [selectionBase] and [selectionExtent] as
  /// -1, if so 0 assigned to the [baseOffset] and [extentOffset]. -1 is not a
  /// valid selection range for input DOM elements.
  factory EditingState.fromFrameworkMessage(Map<String, dynamic> flutterEditingState) {
    final String? text = flutterEditingState.tryString('text');

    final int selectionBase = flutterEditingState.readInt('selectionBase');
    final int selectionExtent = flutterEditingState.readInt('selectionExtent');

    final int? composingBase = flutterEditingState.tryInt('composingBase');
    final int? composingExtent = flutterEditingState.tryInt('composingExtent');

    return EditingState(
      text: text,
      baseOffset: selectionBase,
      extentOffset: selectionExtent,
      composingBaseOffset: composingBase ?? -1,
      composingExtentOffset: composingExtent ?? -1,
    );
  }

  /// Creates an [EditingState] instance using values from the editing element
  /// in the DOM.
  ///
  /// [domElement] can be a [InputElement] or a [TextAreaElement] depending on
  /// the [InputType] of the text field.
  factory EditingState.fromDomElement(DomHTMLElement? domElement) {
    if (domInstanceOfString(domElement, 'HTMLInputElement')) {
      final DomHTMLInputElement element = domElement! as DomHTMLInputElement;
      if (element.selectionDirection == 'backward') {
        return EditingState(
          text: element.value,
          baseOffset: element.selectionEnd?.toInt(),
          extentOffset: element.selectionStart?.toInt(),
        );
      } else {
        return EditingState(
          text: element.value,
          baseOffset: element.selectionStart?.toInt(),
          extentOffset: element.selectionEnd?.toInt(),
        );
      }
    } else if (domInstanceOfString(domElement, 'HTMLTextAreaElement')) {
      final DomHTMLTextAreaElement element = domElement! as DomHTMLTextAreaElement;
      if (element.selectionDirection == 'backward') {
        return EditingState(
          text: element.value,
          baseOffset: element.selectionEnd?.toInt(),
          extentOffset: element.selectionStart?.toInt(),
        );
      } else {
        return EditingState(
          text: element.value,
          baseOffset: element.selectionStart?.toInt(),
          extentOffset: element.selectionEnd?.toInt(),
        );
      }
    } else {
      throw UnsupportedError('Initialized with unsupported input type');
    }
  }

  // Pick the smallest selection index for base.
  int get minOffset => math.min(baseOffset ?? 0, extentOffset ?? 0);
  // Pick the greatest selection index for extent.
  int get maxOffset => math.max(baseOffset ?? 0, extentOffset ?? 0);

  EditingState copyWith({
    String? text,
    int? baseOffset,
    int? extentOffset,
    int? composingBaseOffset,
    int? composingExtentOffset,
  }) {
    return EditingState(
      text: text ?? this.text,
      baseOffset: baseOffset ?? this.baseOffset,
      extentOffset: extentOffset ?? this.extentOffset,
      composingBaseOffset: composingBaseOffset ?? this.composingBaseOffset,
      composingExtentOffset: composingExtentOffset ?? this.composingExtentOffset,
    );
  }

  /// The counterpart of [EditingState.fromFrameworkMessage]. It generates a Map that
  /// can be sent to Flutter.
  // TODO(mdebbar): Should we get `selectionAffinity` and other properties from flutter's editing state?
  Map<String, dynamic> toFlutter() => <String, dynamic>{
    'text': text,
    'selectionBase': baseOffset,
    'selectionExtent': extentOffset,
    'composingBase': composingBaseOffset,
    'composingExtent': composingExtentOffset,
  };

  /// The current text being edited.
  final String? text;

  /// The offset at which the text selection originates.
  final int? baseOffset;

  /// The offset at which the text selection terminates.
  final int? extentOffset;

  /// The offset at which [CompositionAwareMixin.composingText] begins, if any.
  final int composingBaseOffset;

  /// The offset at which [CompositionAwareMixin.composingText] terminates, if any.
  final int composingExtentOffset;

  /// Whether the current editing state is valid or not.
  bool get isValid => baseOffset! >= 0 && extentOffset! >= 0;

  @override
  int get hashCode =>
      Object.hash(text, baseOffset, extentOffset, composingBaseOffset, composingExtentOffset);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (runtimeType != other.runtimeType) {
      return false;
    }
    return other is EditingState &&
        other.text == text &&
        other.minOffset == minOffset &&
        other.maxOffset == maxOffset &&
        other.composingBaseOffset == composingBaseOffset &&
        other.composingExtentOffset == composingExtentOffset;
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      result =
          'EditingState("$text", base:$baseOffset, extent:$extentOffset, composingBase:$composingBaseOffset, composingExtent:$composingExtentOffset)';
      return true;
    }());
    return result;
  }

  /// Sets the selection values of a DOM element using this [EditingState].
  ///
  /// [domElement] can be a [InputElement] or a [TextAreaElement] depending on
  /// the [InputType] of the text field.
  ///
  /// This should only be used by focused elements only, because only focused
  /// elements can have their text selection range set. Attempting to set
  /// selection range on a non-focused element will cause it to request focus.
  ///
  /// See also:
  ///
  ///  * [applyTextToDomElement], which is used for non-focused elements.
  void applyToDomElement(DomHTMLElement? domElement) {
    if (domInstanceOfString(domElement, 'HTMLInputElement')) {
      final DomHTMLInputElement element = domElement! as DomHTMLInputElement;
      element.value = text;
      element.setSelectionRange(minOffset, maxOffset);
    } else if (domInstanceOfString(domElement, 'HTMLTextAreaElement')) {
      final DomHTMLTextAreaElement element = domElement! as DomHTMLTextAreaElement;
      element.value = text;
      element.setSelectionRange(minOffset, maxOffset);
    } else {
      throw UnsupportedError(
        'Unsupported DOM element type: <${domElement?.tagName}> (${domElement.runtimeType})',
      );
    }
  }

  /// Applies the [text] to the [domElement].
  ///
  /// This is used by non-focused elements.
  ///
  /// See also:
  ///
  ///  * [applyToDomElement], which is used for focused elements.
  void applyTextToDomElement(DomHTMLElement? domElement) {
    if (domInstanceOfString(domElement, 'HTMLInputElement')) {
      final DomHTMLInputElement element = domElement! as DomHTMLInputElement;
      element.value = text;
    } else if (domInstanceOfString(domElement, 'HTMLTextAreaElement')) {
      final DomHTMLTextAreaElement element = domElement! as DomHTMLTextAreaElement;
      element.value = text;
    } else {
      throw UnsupportedError('Unsupported DOM element type');
    }
  }
}

/// Controls the appearance of the input control being edited.
///
/// For example, [inputType] determines whether we should use `<input>` or
/// `<textarea>` as a backing DOM element.
///
/// This corresponds to Flutter's [TextInputConfiguration].
class InputConfiguration {
  InputConfiguration({
    required this.viewId,
    this.inputType = EngineInputType.text,
    this.inputAction = 'TextInputAction.done',
    this.obscureText = false,
    this.readOnly = false,
    this.autocorrect = true,
    this.textCapitalization = const TextCapitalizationConfig.defaultCapitalization(),
    this.autofill,
    this.autofillGroup,
    this.enableDeltaModel = false,
  });

  InputConfiguration.fromFrameworkMessage(Map<String, dynamic> flutterInputConfiguration)
    : viewId = flutterInputConfiguration.tryInt('viewId') ?? kImplicitViewId,
      inputType = EngineInputType.fromName(
        flutterInputConfiguration.readJson('inputType').readString('name'),
        isDecimal: flutterInputConfiguration.readJson('inputType').tryBool('decimal') ?? false,
        isMultiline:
            flutterInputConfiguration.readJson('inputType').tryBool('isMultiline') ?? false,
      ),
      inputAction = flutterInputConfiguration.tryString('inputAction') ?? 'TextInputAction.done',
      obscureText = flutterInputConfiguration.tryBool('obscureText') ?? false,
      readOnly = flutterInputConfiguration.tryBool('readOnly') ?? false,
      autocorrect = flutterInputConfiguration.tryBool('autocorrect') ?? true,
      textCapitalization = TextCapitalizationConfig.fromInputConfiguration(
        flutterInputConfiguration.readString('textCapitalization'),
      ),
      autofill =
          flutterInputConfiguration.containsKey('autofill')
              ? AutofillInfo.fromFrameworkMessage(flutterInputConfiguration.readJson('autofill'))
              : null,
      autofillGroup = EngineAutofillForm.fromFrameworkMessage(
        flutterInputConfiguration.tryInt('viewId') ?? kImplicitViewId,
        flutterInputConfiguration.tryJson('autofill'),
        flutterInputConfiguration.tryList('fields'),
      ),
      enableDeltaModel = flutterInputConfiguration.tryBool('enableDeltaModel') ?? false;

  /// The ID of the view that contains the text field.
  final int viewId;

  /// The type of information being edited in the input control.
  final EngineInputType inputType;

  /// The default action for the input field.
  final String inputAction;

  /// Whether the text field can be edited or not.
  ///
  /// Defaults to false.
  final bool readOnly;

  /// Whether to hide the text being edited.
  final bool obscureText;

  /// Whether to enable autocorrection.
  ///
  /// Definition of autocorrect can be found in:
  /// https://developer.mozilla.org/en-US/docs/Web/HTML/Element/input
  ///
  /// For future manual tests, note that autocorrect is an attribute only
  /// supported by Safari.
  final bool autocorrect;

  final bool enableDeltaModel;

  final AutofillInfo? autofill;

  final EngineAutofillForm? autofillGroup;

  final TextCapitalizationConfig textCapitalization;
}

typedef OnChangeCallback =
    void Function(EditingState? editingState, TextEditingDeltaState? editingDeltaState);
typedef OnActionCallback = void Function(String? inputAction);

/// Provides HTML DOM functionality for editable text.
///
/// A concrete implementation is picked at runtime based on the current
/// operating system, web browser, and accessibility mode.
abstract class TextEditingStrategy {
  void initializeTextEditing(
    InputConfiguration inputConfig, {
    required OnChangeCallback onChange,
    required OnActionCallback onAction,
  });

  /// Sets the initial placement of the DOM element on the UI.
  ///
  /// The element must be located exactly in the same place with the editable
  /// widget. However, its contents and cursor will be invisible.
  ///
  /// Users can interact with the element and use the functionality of the
  /// right-click menu, such as copy, paste, cut, select, translate, etc.
  void initializeElementPlacement();

  /// Register event listeners to the DOM element.
  ///
  /// These event listener will be removed in [disable].
  void addEventHandlers();

  /// Update the element's position.
  ///
  /// The position will be updated everytime Flutter Framework sends
  /// 'TextInput.setEditableSizeAndTransform' message.
  void updateElementPlacement(EditableTextGeometry geometry);

  /// Set editing state of the element.
  ///
  /// This includes text and selection relelated states. The editing state will
  /// be updated everytime Flutter Framework sends 'TextInput.setEditingState'
  /// message.
  void setEditingState(EditingState editingState);

  /// Set style to the native DOM element used for text editing.
  void updateElementStyle(EditableTextStyle style);

  /// Disables the element so it's no longer used for text editing.
  ///
  /// Calling [disable] also removes any registered event listeners.
  void disable();
}

/// A [TextEditingStrategy] that places its [domElement] assuming no
/// prior transform or sizing is applied to it.
///
/// This implementation is used by text editables when semantics is not
/// enabled. With semantics enabled the placement is provided by the semantics
/// tree.
class GloballyPositionedTextEditingStrategy extends DefaultTextEditingStrategy {
  GloballyPositionedTextEditingStrategy(super.owner);

  @override
  void placeElement() {
    geometry?.applyToDomElement(activeDomElement);
    if (hasAutofillGroup) {
      placeForm();
      // Set the last editing state if it exists, this is critical for a
      // users ongoing work to continue uninterrupted when there is an update to
      // the transform.
      lastEditingState?.applyToDomElement(domElement);
      // On Chrome, when a form is focused, it opens an autofill menu
      // immediately.
      // Flutter framework sends `setEditableSizeAndTransform` for informing
      // the engine about the location of the text field. This call will
      // arrive after `show` call.
      // Therefore on Chrome we place the element when
      //  `setEditableSizeAndTransform` method is called and focus on the form
      // only after placing it to the correct position. Hence autofill menu
      // does not appear on top-left of the page.
      // Refocus on the elements after applying the geometry.
      focusedFormElement!.focusWithoutScroll();
      moveFocusToActiveDomElement();
    }
  }
}

/// A [TextEditingStrategy] for Safari Desktop Browser.
///
/// It places its [domElement] assuming no prior transform or sizing is applied
/// to it.
///
/// In case of an autofill enabled form, it does not append the form element
/// to the DOM, until the geometry information is updated.
///
/// This implementation is used by text editables when semantics is not
/// enabled. With semantics enabled the placement is provided by the semantics
/// tree.
class SafariDesktopTextEditingStrategy extends DefaultTextEditingStrategy {
  SafariDesktopTextEditingStrategy(super.owner);

  /// Appending an element on the DOM for Safari Desktop Browser.
  ///
  /// This method is only called when geometry information is updated by
  /// 'TextInput.setEditableSizeAndTransform' message.
  ///
  /// This method is similar to the [GloballyPositionedTextEditingStrategy].
  /// The only part different: this method does not call `super.placeElement()`,
  /// which in current state calls `domElement.focusWithoutScroll()`.
  ///
  /// Making an extra `focus` request causes flickering in Safari.
  @override
  void placeElement() {
    geometry?.applyToDomElement(activeDomElement);
    if (hasAutofillGroup) {
      placeForm();
      // Set the last editing state if it exists, this is critical for a
      // users ongoing work to continue uninterrupted when there is an update to
      // the transform.
      // If domElement is not focused cursor location will not be correct.
      moveFocusToActiveDomElement();
      lastEditingState?.applyToDomElement(activeDomElement);
    }
  }

  @override
  void initializeElementPlacement() {
    if (geometry != null) {
      placeElement();
    }
    moveFocusToActiveDomElement();
  }
}

/// Class implementing the default editing strategies for text editing.
///
/// This class uses a DOM element to provide text editing capabilities.
///
/// The backing DOM element could be one of:
///
/// 1. `<input>`.
/// 2. `<textarea>`.
/// 3. `<span contenteditable="true">`.
///
/// This class includes all the default behaviour for an editing element as
/// well as the common properties such as [domElement].
///
/// Strategies written for different form factors and browsers should extend
/// this class instead of extending the interface [TextEditingStrategy]. In
/// particular, a concrete implementation is expected to override
/// [placeElement] that places the DOM element accordingly. The default
/// implementation of [placeElement] does not position the element.
///
/// Unless a formfactor/browser requires specific implementation for a specific
/// strategy the methods in this class should be used.
abstract class DefaultTextEditingStrategy
    with CompositionAwareMixin
    implements TextEditingStrategy {
  DefaultTextEditingStrategy(this.owner);

  final HybridTextEditing owner;

  bool isEnabled = false;

  /// The DOM element used for editing, if any.
  DomHTMLElement? domElement;

  /// Same as [domElement] but null-checked.
  ///
  /// This must only be called in places that know for sure that a DOM element
  /// is currently available for editing.
  DomHTMLElement get activeDomElement {
    assert(
      domElement != null,
      'The DOM element of this text editing strategy is not currently active.',
    );
    return domElement!;
  }

  /// The [FlutterView] in which [activeDomElement] is contained.
  EngineFlutterView? get activeDomElementView => _viewForElement(activeDomElement);

  EngineFlutterView? _viewForElement(DomElement element) =>
      EnginePlatformDispatcher.instance.viewManager.findViewForElement(element);

  late InputConfiguration inputConfiguration;
  EditingState? lastEditingState;

  TextEditingDeltaState? _editingDeltaState;
  TextEditingDeltaState get editingDeltaState {
    _editingDeltaState ??= TextEditingDeltaState(oldText: lastEditingState!.text!);
    return _editingDeltaState!;
  }

  /// Styles associated with the editable text.
  EditableTextStyle? style;

  /// Size and transform of the editable text on the page.
  EditableTextGeometry? geometry;

  OnChangeCallback? onChange;
  OnActionCallback? onAction;

  final List<DomSubscription> subscriptions = <DomSubscription>[];

  bool get hasAutofillGroup => inputConfiguration.autofillGroup != null;

  /// Whether the focused input element is part of a form.
  bool get appendedToForm => _appendedToForm;
  bool _appendedToForm = false;

  DomHTMLFormElement? get focusedFormElement => inputConfiguration.autofillGroup?.formElement;

  @override
  void initializeTextEditing(
    InputConfiguration inputConfig, {
    required OnChangeCallback onChange,
    required OnActionCallback onAction,
  }) {
    assert(!isEnabled);

    // The -1 tab index value makes this element not reachable by keyboard.
    domElement = inputConfig.inputType.createDomElement()..tabIndex = -1;
    applyConfiguration(inputConfig);

    _setStaticStyleAttributes(activeDomElement);
    style?.applyToDomElement(activeDomElement);

    if (!hasAutofillGroup) {
      // If there is an Autofill Group the `FormElement`, it will be appended to the
      // DOM later, when the first location information arrived.
      // Otherwise, on Blink based Desktop browsers, the autofill menu appears
      // on top left of the screen.
      _insertEditingElementInView(activeDomElement, inputConfig.viewId);
      _appendedToForm = false;
    }

    initializeElementPlacement();

    isEnabled = true;
    this.onChange = onChange;
    this.onAction = onAction;
  }

  void applyConfiguration(InputConfiguration config) {
    inputConfiguration = config;

    if (config.readOnly) {
      activeDomElement.setAttribute('readonly', 'readonly');
    } else {
      activeDomElement.removeAttribute('readonly');
    }

    if (config.obscureText) {
      activeDomElement.setAttribute('type', 'password');
    }

    if (config.inputType.inputmodeAttribute == 'none') {
      activeDomElement.setAttribute('inputmode', 'none');
    }

    final EngineInputAction action = EngineInputAction.fromName(config.inputAction);
    action.configureInputAction(activeDomElement);

    final AutofillInfo? autofill = config.autofill;
    if (autofill != null) {
      autofill.applyToDomElement(activeDomElement, focusedElement: true);
    } else {
      activeDomElement.setAttribute('autocomplete', 'off');
      // When the new input configuration contains a different view ID, we need
      // to move the input element to the new view.
      _ensureEditingElementInView(activeDomElement, inputConfiguration.viewId);
    }

    final String autocorrectValue = config.autocorrect ? 'on' : 'off';
    activeDomElement.setAttribute('autocorrect', autocorrectValue);
  }

  @override
  void initializeElementPlacement() {
    placeElement();
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

    subscriptions.add(DomSubscription(activeDomElement, 'beforeinput', handleBeforeInput));

    if (this is! SafariDesktopTextEditingStrategy) {
      // handleBlur causes Safari to reopen autofill dialogs after autofill,
      // so we don't attach the listener there.
      subscriptions.add(DomSubscription(activeDomElement, 'blur', handleBlur));
    }

    addCompositionEventHandlers(activeDomElement);

    preventDefaultForMouseEvents();
  }

  @override
  void updateElementPlacement(EditableTextGeometry textGeometry) {
    geometry = textGeometry;
    if (isEnabled) {
      // On updates, we shouldn't go through the entire placeElement() flow if
      // we are in the middle of IME composition, otherwise we risk interrupting it.
      // Geometry updates occur when a multiline input expands or contracts. If
      // we are in the middle of composition, we should just update the geometry.
      // See: https://github.com/flutter/flutter/issues/98817
      if (composingText != null) {
        geometry?.applyToDomElement(activeDomElement);
      } else {
        placeElement();
      }
    }
  }

  @override
  void updateElementStyle(EditableTextStyle textStyle) {
    style = textStyle;
    if (isEnabled) {
      textStyle.applyToDomElement(activeDomElement);
    }
  }

  @override
  void disable() {
    assert(isEnabled);

    isEnabled = false;
    lastEditingState = null;
    _editingDeltaState = null;
    style = null;
    geometry = null;

    for (int i = 0; i < subscriptions.length; i++) {
      subscriptions[i].cancel();
    }
    subscriptions.clear();
    removeCompositionEventHandlers(activeDomElement);

    // If focused element is a part of a form, it needs to stay on the DOM
    // until the autofill context of the form is finalized.
    // More details on `TextInput.finishAutofillContext` call.
    if (_appendedToForm && inputConfiguration.autofillGroup?.formElement != null) {
      _styleAutofillElements(activeDomElement, isOffScreen: true);
      inputConfiguration.autofillGroup?.storeForm();
      EnginePlatformDispatcher.instance.viewManager.safeBlur(activeDomElement);
    } else {
      EnginePlatformDispatcher.instance.viewManager.safeRemove(activeDomElement);
    }
    domElement = null;
  }

  @override
  void setEditingState(EditingState? editingState) {
    lastEditingState = editingState;
    if (!isEnabled || !editingState!.isValid) {
      return;
    }
    lastEditingState!.applyToDomElement(domElement);
  }

  void placeElement() {
    moveFocusToActiveDomElement();
  }

  void placeForm() {
    inputConfiguration.autofillGroup!.placeForm(activeDomElement);
    _appendedToForm = true;
  }

  void handleChange(DomEvent event) {
    assert(isEnabled);

    EditingState newEditingState = EditingState.fromDomElement(activeDomElement);
    newEditingState = determineCompositionState(newEditingState);

    TextEditingDeltaState? newTextEditingDeltaState;
    if (inputConfiguration.enableDeltaModel) {
      editingDeltaState.composingOffset = newEditingState.composingBaseOffset;
      editingDeltaState.composingExtent = newEditingState.composingExtentOffset;
      newTextEditingDeltaState = TextEditingDeltaState.inferDeltaState(
        newEditingState,
        lastEditingState,
        editingDeltaState,
      );
    }

    if (newEditingState != lastEditingState) {
      lastEditingState = newEditingState;
      _editingDeltaState = newTextEditingDeltaState;
      onChange!(lastEditingState, _editingDeltaState);
    }
    // Flush delta state.
    _editingDeltaState = null;
  }

  void handleBeforeInput(DomEvent event) {
    // In some cases the beforeinput event is not fired such as when the selection
    // of a text field is updated. In this case only the oninput event is fired.
    // We still want a delta generated in these cases so we can properly update
    // the selection. We begin to set the deltaStart and deltaEnd in beforeinput
    // because a change in the selection will not have a delta range, it will only
    // have a baseOffset and extentOffset. If these are set inside of inferDeltaState
    // then the method will incorrectly report a deltaStart and deltaEnd for a non
    // text update delta.
    final String? eventData = getJsProperty<void>(event, 'data') as String?;
    final String? inputType = getJsProperty<void>(event, 'inputType') as String?;

    if (inputType != null) {
      final bool isSelectionInverted =
          lastEditingState!.baseOffset! > lastEditingState!.extentOffset!;
      final int deltaOffset =
          isSelectionInverted ? lastEditingState!.baseOffset! : lastEditingState!.extentOffset!;
      if (inputType.contains('delete')) {
        // The deltaStart is set in handleChange because there is where we get access
        // to the new selection baseOffset which is our new deltaStart.
        editingDeltaState.deltaText = '';
        editingDeltaState.deltaEnd = deltaOffset;
      } else if (inputType == 'insertLineBreak') {
        // event.data is null on a line break, so we manually set deltaText as a line break by setting it to '\n'.
        editingDeltaState.deltaText = '\n';
        editingDeltaState.deltaStart = deltaOffset;
        editingDeltaState.deltaEnd = deltaOffset;
      } else if (eventData != null) {
        // When event.data is not null we will begin by considering this delta as an insertion
        // at the selection extentOffset. This may change due to logic in handleChange to handle
        // composition and other IME behaviors.
        editingDeltaState.deltaText = eventData;
        editingDeltaState.deltaStart = deltaOffset;
        editingDeltaState.deltaEnd = deltaOffset;
      }
    }
  }

  void handleBlur(DomEvent event) {
    event as DomFocusEvent;

    final DomElement? willGainFocusElement = event.relatedTarget as DomElement?;
    if (willGainFocusElement == null ||
        _viewForElement(willGainFocusElement) == activeDomElementView) {
      moveFocusToActiveDomElement();
    }
  }

  void maybeSendAction(DomEvent e) {
    if (domInstanceOfString(e, 'KeyboardEvent')) {
      final DomKeyboardEvent event = e as DomKeyboardEvent;
      if (event.keyCode == _kReturnKeyCode) {
        onAction!(inputConfiguration.inputAction);
        if (inputConfiguration.inputType is MultilineInputType &&
            inputConfiguration.inputAction == 'TextInputAction.newline') {
          return;
        }
        // Prevent the browser from inserting a new line.
        event.preventDefault();
      }
    }
  }

  /// Enables the element so it can be used to edit text.
  ///
  /// Register [callback] so that it gets invoked whenever any change occurs in
  /// the text editing element.
  ///
  /// Changes could be:
  /// - Text changes, or
  /// - Selection changes.
  void enable(
    InputConfiguration inputConfig, {
    required OnChangeCallback onChange,
    required OnActionCallback onAction,
  }) {
    assert(!isEnabled);

    initializeTextEditing(inputConfig, onChange: onChange, onAction: onAction);

    addEventHandlers();

    if (lastEditingState != null) {
      setEditingState(lastEditingState);
    }

    // Re-focuses after setting editing state.
    moveFocusToActiveDomElement();
  }

  /// Prevent default behavior for mouse down, up and move.
  ///
  /// When normal mouse events are not prevented, mouse selection
  /// conflicts with selection sent from the framework, which creates
  /// flickering during selection by mouse.
  ///
  /// On mobile browsers, mouse events are sent after a touch event,
  /// see: https://bugs.chromium.org/p/chromium/issues/detail?id=119216#c11.
  void preventDefaultForMouseEvents() {
    subscriptions.add(
      DomSubscription(activeDomElement, 'mousedown', (DomEvent event) {
        event.preventDefault();
      }),
    );

    subscriptions.add(
      DomSubscription(activeDomElement, 'mouseup', (DomEvent event) {
        event.preventDefault();
      }),
    );

    subscriptions.add(
      DomSubscription(activeDomElement, 'mousemove', (DomEvent event) {
        event.preventDefault();
      }),
    );
  }

  /// Moves the focus to the [activeDomElement].
  void moveFocusToActiveDomElement() {
    activeDomElement.focusWithoutScroll();
  }
}

/// IOS/Safari behaviour for text editing.
///
/// In iOS, the virtual keyboard might shifts the screen up to make input
/// visible depending on the location of the focused input element.
///
/// Due to this [initializeElementPlacement] and [updateElementPlacement]
/// strategies are different.
///
/// [disable] is also different since the [_positionInputElementTimer]
/// also needs to be cleaned.
///
/// inputmodeAttribute needs to be set for mobile devices. Due to this
/// [initializeTextEditing] is different.
class IOSTextEditingStrategy extends GloballyPositionedTextEditingStrategy {
  IOSTextEditingStrategy(super.owner);

  /// Timer that times when to set the location of the input text.
  ///
  /// This is only used for iOS. In iOS, virtual keyboard shifts the screen.
  /// There is no callback to know if the keyboard is up and how much the screen
  /// has shifted. Therefore instead of listening to the shift and passing this
  /// information to Flutter Framework, we are trying to stop the shift.
  ///
  /// In iOS, the virtual keyboard shifts the screen up if the focused input
  /// element is under the keyboard or very close to the keyboard. Before the
  /// focus is called we are positioning it offscreen. The location of the input
  /// in iOS is set to correct place, 100ms after focus. We use this timer for
  /// timing this delay.
  Timer? _positionInputElementTimer;
  static const Duration _delayBeforePlacement = Duration(milliseconds: 100);

  /// Whether or not the input element can be positioned at this point in time.
  ///
  /// This is currently only used in iOS. It's set to false before focusing the
  /// input field, and set back to true after a short timer. We do this because
  /// if the input field is positioned before focus, it could be pushed to an
  /// incorrect position by the virtual keyboard.
  ///
  /// See:
  ///
  /// * [_delayBeforePlacement] which controls how long to wait before
  ///   positioning the input field.
  bool _canPosition = true;

  @override
  void initializeTextEditing(
    InputConfiguration inputConfig, {
    required OnChangeCallback onChange,
    required OnActionCallback onAction,
  }) {
    super.initializeTextEditing(inputConfig, onChange: onChange, onAction: onAction);
    inputConfig.inputType.configureInputMode(activeDomElement);
    if (hasAutofillGroup) {
      placeForm();
    }
    inputConfig.textCapitalization.setAutocapitalizeAttribute(activeDomElement);
  }

  @override
  void initializeElementPlacement() {
    /// Position the element outside of the page before focusing on it. This is
    /// useful for not triggering a scroll when iOS virtual keyboard is
    /// coming up.
    activeDomElement.style.transform = 'translate(${offScreenOffset}px, ${offScreenOffset}px)';

    _canPosition = false;
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

    subscriptions.add(DomSubscription(activeDomElement, 'beforeinput', handleBeforeInput));

    subscriptions.add(DomSubscription(activeDomElement, 'blur', handleBlur));

    addCompositionEventHandlers(activeDomElement);

    // Position the DOM element after it is focused.
    subscriptions.add(
      DomSubscription(activeDomElement, 'focus', (_) {
        // Cancel previous timer if exists.
        _schedulePlacement();
      }),
    );

    _addTapListener();
  }

  @override
  void updateElementPlacement(EditableTextGeometry textGeometry) {
    geometry = textGeometry;
    if (isEnabled && _canPosition) {
      placeElement();
    }
  }

  @override
  void disable() {
    super.disable();
    _positionInputElementTimer?.cancel();
    _positionInputElementTimer = null;
  }

  /// On iOS long press works differently than a single tap.
  ///
  /// On a normal tap the virtual keyboard comes up and users can enter text
  /// using the keyboard.
  ///
  /// The long press on the other hand focuses on the element without bringing
  /// up the virtual keyboard. It allows the users to modify the field by using
  /// copy/cut/select/paste etc.
  ///
  /// After a long press [domElement] is positioned to the correct place. If the
  /// user later single-tap on the [domElement] the virtual keyboard will come
  /// and might shift the page up.
  ///
  /// In order to prevent this shift, on a `click` event the position of the
  /// element is again set somewhere outside of the page and
  /// [_positionInputElementTimer] timer is restarted. The element will be
  /// placed to its correct position after [_delayBeforePlacement].
  void _addTapListener() {
    subscriptions.add(
      DomSubscription(activeDomElement, 'click', (_) {
        // Check if the element is already positioned. If not this does not fall
        // under `The user was using the long press, now they want to enter text
        // via keyboard` journey.
        if (_canPosition) {
          // Re-place the element somewhere outside of the screen.
          initializeElementPlacement();

          // Re-configure the timer to place the element.
          _schedulePlacement();
        }
      }),
    );
  }

  void _schedulePlacement() {
    _positionInputElementTimer?.cancel();
    _positionInputElementTimer = Timer(_delayBeforePlacement, () {
      _canPosition = true;
      placeElement();
    });
  }

  @override
  void placeElement() {
    moveFocusToActiveDomElement();
    geometry?.applyToDomElement(activeDomElement);
  }
}

/// Android behaviour for text editing.
///
/// inputmodeAttribute needs to be set for mobile devices. Due to this
/// [initializeTextEditing] is different.
///
/// Keyboard acts differently than other devices. [addEventHandlers] handles
/// this case as an extra.
class AndroidTextEditingStrategy extends GloballyPositionedTextEditingStrategy {
  AndroidTextEditingStrategy(super.owner);

  @override
  void initializeTextEditing(
    InputConfiguration inputConfig, {
    required OnChangeCallback onChange,
    required OnActionCallback onAction,
  }) {
    super.initializeTextEditing(inputConfig, onChange: onChange, onAction: onAction);
    inputConfig.inputType.configureInputMode(activeDomElement);
    if (hasAutofillGroup) {
      placeForm();
    } else {
      _insertEditingElementInView(activeDomElement, inputConfig.viewId);
    }
    inputConfig.textCapitalization.setAutocapitalizeAttribute(activeDomElement);
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

    subscriptions.add(DomSubscription(activeDomElement, 'beforeinput', handleBeforeInput));

    subscriptions.add(DomSubscription(activeDomElement, 'blur', handleBlur));

    addCompositionEventHandlers(activeDomElement);

    preventDefaultForMouseEvents();
  }

  @override
  void placeElement() {
    moveFocusToActiveDomElement();
    geometry?.applyToDomElement(activeDomElement);
  }
}

/// Firefox behaviour for text editing.
///
/// Selections are different in Firefox. [addEventHandlers] strategy is
/// impelemented diefferently in Firefox.
class FirefoxTextEditingStrategy extends GloballyPositionedTextEditingStrategy {
  FirefoxTextEditingStrategy(super.owner);

  @override
  void initializeTextEditing(
    InputConfiguration inputConfig, {
    required OnChangeCallback onChange,
    required OnActionCallback onAction,
  }) {
    super.initializeTextEditing(inputConfig, onChange: onChange, onAction: onAction);
    if (hasAutofillGroup) {
      placeForm();
    }
  }

  @override
  void addEventHandlers() {
    if (inputConfiguration.autofillGroup != null) {
      subscriptions.addAll(inputConfiguration.autofillGroup!.addInputEventListeners());
    }

    // Subscribe to text and selection changes.
    subscriptions.add(DomSubscription(activeDomElement, 'input', handleChange));

    subscriptions.add(DomSubscription(activeDomElement, 'keydown', maybeSendAction));

    subscriptions.add(DomSubscription(activeDomElement, 'beforeinput', handleBeforeInput));

    addCompositionEventHandlers(activeDomElement);

    // Detects changes in text selection.
    //
    // In Firefox, when cursor moves, neither selectionChange nor onInput
    // events are triggered. We are listening to keyup event. Selection start,
    // end values are used to decide if the text cursor moved.
    //
    // Specific keycodes are not checked since users/applications can bind
    // their own keys to move the text cursor.
    // Decides if the selection has changed (cursor moved) compared to the
    // previous values.
    //
    // After each keyup, the start/end values of the selection is compared to
    // the previously saved editing state.
    subscriptions.add(
      DomSubscription(activeDomElement, 'keyup', (DomEvent event) {
        handleChange(event);
      }),
    );

    // In Firefox the context menu item "Select All" does not work without
    // listening to onSelect. On the other browsers onSelectionChange is
    // enough for covering "Select All" functionality.
    subscriptions.add(DomSubscription(activeDomElement, 'select', handleChange));

    subscriptions.add(DomSubscription(activeDomElement, 'blur', handleBlur));

    preventDefaultForMouseEvents();
  }

  @override
  void placeElement() {
    moveFocusToActiveDomElement();
    geometry?.applyToDomElement(activeDomElement);
    // Set the last editing state if it exists, this is critical for a
    // users ongoing work to continue uninterrupted when there is an update to
    // the transform.
    lastEditingState?.applyToDomElement(activeDomElement);
  }
}

/// Base class for all `TextInput` commands sent through the `flutter/textinput`
/// channel.
@immutable
abstract class TextInputCommand {
  const TextInputCommand();

  /// Executes the logic for this command.
  void run(HybridTextEditing textEditing);
}

/// Responds to the 'TextInput.setClient' message.
class TextInputSetClient extends TextInputCommand {
  const TextInputSetClient({required this.clientId, required this.configuration});

  final int clientId;
  final InputConfiguration configuration;

  @override
  void run(HybridTextEditing textEditing) {
    final bool clientIdChanged = textEditing._clientId != null && textEditing._clientId != clientId;
    if (clientIdChanged && textEditing.isEditing) {
      // We're connecting a new client. Any pending command for the previous client
      // are irrelevant at this point.
      textEditing.stopEditing();
    }
    textEditing._clientId = clientId;
    textEditing.configuration = configuration;
  }
}

/// Creates the text editing strategy used in non-a11y mode.
DefaultTextEditingStrategy createDefaultTextEditingStrategy(HybridTextEditing textEditing) {
  DefaultTextEditingStrategy strategy;

  if (ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs) {
    strategy = IOSTextEditingStrategy(textEditing);
  } else if (ui_web.browser.operatingSystem == ui_web.OperatingSystem.android) {
    strategy = AndroidTextEditingStrategy(textEditing);
  } else if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
    strategy = SafariDesktopTextEditingStrategy(textEditing);
  } else if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
    strategy = FirefoxTextEditingStrategy(textEditing);
  } else {
    strategy = GloballyPositionedTextEditingStrategy(textEditing);
  }

  return strategy;
}

/// Responds to the 'TextInput.updateConfig' message.
class TextInputUpdateConfig extends TextInputCommand {
  const TextInputUpdateConfig();

  @override
  void run(HybridTextEditing textEditing) {
    textEditing.strategy.applyConfiguration(textEditing.configuration!);
  }
}

/// Responds to the 'TextInput.setEditingState' message.
class TextInputSetEditingState extends TextInputCommand {
  const TextInputSetEditingState({required this.state});

  final EditingState state;

  @override
  void run(HybridTextEditing textEditing) {
    textEditing.strategy.setEditingState(state);
  }
}

/// Responds to the 'TextInput.show' message.
class TextInputShow extends TextInputCommand {
  const TextInputShow();

  @override
  void run(HybridTextEditing textEditing) {
    if (!textEditing.isEditing) {
      textEditing._startEditing();
    }
  }
}

/// Responds to the 'TextInput.setEditableSizeAndTransform' message.
class TextInputSetEditableSizeAndTransform extends TextInputCommand {
  const TextInputSetEditableSizeAndTransform({required this.geometry});

  final EditableTextGeometry geometry;

  @override
  void run(HybridTextEditing textEditing) {
    textEditing.strategy.updateElementPlacement(geometry);
  }
}

/// Responds to the 'TextInput.setStyle' message.
class TextInputSetStyle extends TextInputCommand {
  const TextInputSetStyle({required this.style});

  final EditableTextStyle style;

  @override
  void run(HybridTextEditing textEditing) {
    textEditing.strategy.updateElementStyle(style);
  }
}

/// Responds to the 'TextInput.clearClient' message.
class TextInputClearClient extends TextInputCommand {
  const TextInputClearClient();

  @override
  void run(HybridTextEditing textEditing) {
    if (textEditing.isEditing) {
      textEditing.stopEditing();
    }
  }
}

/// Responds to the 'TextInput.hide' message.
class TextInputHide extends TextInputCommand {
  const TextInputHide();

  @override
  void run(HybridTextEditing textEditing) {
    if (textEditing.isEditing) {
      textEditing.stopEditing();
    }
  }
}

class TextInputSetMarkedTextRect extends TextInputCommand {
  const TextInputSetMarkedTextRect();

  @override
  void run(HybridTextEditing textEditing) {
    // No-op: this message is currently only used on iOS to implement
    // UITextInput.firstRecForRange.
  }
}

class TextInputSetCaretRect extends TextInputCommand {
  const TextInputSetCaretRect();

  @override
  void run(HybridTextEditing textEditing) {
    // No-op: not supported on this platform.
  }
}

class TextInputRequestAutofill extends TextInputCommand {
  const TextInputRequestAutofill();

  @override
  void run(HybridTextEditing textEditing) {
    // No-op: not supported on this platform.
  }
}

class TextInputFinishAutofillContext extends TextInputCommand {
  const TextInputFinishAutofillContext({required this.saveForm});

  final bool saveForm;

  @override
  void run(HybridTextEditing textEditing) {
    // Close the text editing connection. Form is finalizing.
    textEditing.sendTextConnectionClosedToFrameworkIfAny();
    if (saveForm) {
      saveForms();
    }
    // Clean the forms from DOM after submitting them.
    cleanForms();
  }
}

/// Submits the forms currently attached to the DOM.
///
/// Browser will save the information entered to the form.
///
/// Called when the form is finalized with save option `true`.
/// See: https://github.com/flutter/flutter/blob/bf9f3a3dcfea3022f9cf2dfc3ab10b120b48b19d/packages/flutter/lib/src/services/text_input.dart#L1277
void saveForms() {
  formsOnTheDom.forEach((String identifier, DomHTMLFormElement form) {
    final DomHTMLInputElement submitBtn =
        form.getElementsByClassName('submitBtn').first as DomHTMLInputElement;
    submitBtn.click();
  });
}

/// Removes the forms from the DOM.
///
/// Called when the form is finalized.
void cleanForms() {
  for (final DomHTMLFormElement form in formsOnTheDom.values) {
    form.remove();
  }
  formsOnTheDom.clear();
}

/// Translates the message-based communication between the framework and the
/// engine [implementation].
///
/// This class is meant to be used as a singleton.
class TextEditingChannel {
  TextEditingChannel(this.implementation);

  /// Supplies the implementation that responds to the channel messages.
  final HybridTextEditing implementation;

  /// Handles "flutter/textinput" platform messages received from the framework.
  void handleTextInput(ByteData? data, ui.PlatformMessageResponseCallback? callback) {
    const JSONMethodCodec codec = JSONMethodCodec();
    final MethodCall call = codec.decodeMethodCall(data);
    final TextInputCommand command;
    switch (call.method) {
      case 'TextInput.setClient':
        final args = call.arguments! as List<Object?>;
        command = TextInputSetClient(
          clientId: args[0]! as int,
          configuration: InputConfiguration.fromFrameworkMessage(args[1]! as Map<String, Object?>),
        );

      case 'TextInput.updateConfig':
        // Set configuration eagerly because it contains data about the text
        // field used to flush the command queue. However, delaye applying the
        // configuration because the strategy may not be available yet.
        implementation.configuration = InputConfiguration.fromFrameworkMessage(
          call.arguments as Map<String, dynamic>,
        );
        command = const TextInputUpdateConfig();

      case 'TextInput.setEditingState':
        command = TextInputSetEditingState(
          state: EditingState.fromFrameworkMessage(call.arguments as Map<String, dynamic>),
        );

      case 'TextInput.show':
        command = const TextInputShow();

      case 'TextInput.setEditableSizeAndTransform':
        command = TextInputSetEditableSizeAndTransform(
          geometry: EditableTextGeometry.fromFrameworkMessage(
            call.arguments as Map<String, dynamic>,
          ),
        );

      case 'TextInput.setStyle':
        command = TextInputSetStyle(
          style: EditableTextStyle.fromFrameworkMessage(call.arguments as Map<String, dynamic>),
        );

      case 'TextInput.clearClient':
        command = const TextInputClearClient();

      case 'TextInput.hide':
        command = const TextInputHide();

      case 'TextInput.requestAutofill':
        // There's no API to request autofill on the web. Instead we let the
        // browser show autofill options automatically, if available. We
        // therefore simply ignore this message.
        command = const TextInputRequestAutofill();

      case 'TextInput.finishAutofillContext':
        command = TextInputFinishAutofillContext(saveForm: call.arguments as bool);

      case 'TextInput.setMarkedTextRect':
        command = const TextInputSetMarkedTextRect();

      case 'TextInput.setCaretRect':
        command = const TextInputSetCaretRect();

      default:
        if (_debugPrintTextInputCommands) {
          print('Received unknown command on flutter/textinput channel: ${call.method}');
        }
        EnginePlatformDispatcher.instance.replyToPlatformMessage(callback, null);
        return;
    }

    implementation.acceptCommand(command, () {
      EnginePlatformDispatcher.instance.replyToPlatformMessage(
        callback,
        codec.encodeSuccessEnvelope(true),
      );
    });
  }

  /// Sends the 'TextInputClient.updateEditingState' message to the framework.
  void updateEditingState(int? clientId, EditingState? editingState) {
    EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
      'flutter/textinput',
      const JSONMethodCodec().encodeMethodCall(
        MethodCall('TextInputClient.updateEditingState', <dynamic>[
          clientId,
          editingState!.toFlutter(),
        ]),
      ),
      _emptyCallback,
    );
  }

  /// Sends the 'TextInputClient.updateEditingStateWithDeltas' message to the framework.
  void updateEditingStateWithDelta(int? clientId, TextEditingDeltaState? editingDeltaState) {
    EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
      'flutter/textinput',
      const JSONMethodCodec().encodeMethodCall(
        MethodCall('TextInputClient.updateEditingStateWithDeltas', <dynamic>[
          clientId,
          editingDeltaState!.toFlutter(),
        ]),
      ),
      _emptyCallback,
    );
  }

  /// Sends the 'TextInputClient.performAction' message to the framework.
  void performAction(int? clientId, String? inputAction) {
    EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
      'flutter/textinput',
      const JSONMethodCodec().encodeMethodCall(
        MethodCall('TextInputClient.performAction', <dynamic>[clientId, inputAction]),
      ),
      _emptyCallback,
    );
  }

  /// Sends the 'TextInputClient.onConnectionClosed' message to the framework.
  void onConnectionClosed(int? clientId) {
    EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
      'flutter/textinput',
      const JSONMethodCodec().encodeMethodCall(
        MethodCall('TextInputClient.onConnectionClosed', <dynamic>[clientId]),
      ),
      _emptyCallback,
    );
  }
}

/// Text editing singleton.
final HybridTextEditing textEditing = HybridTextEditing();

/// Map for storing forms left attached on the DOM.
///
/// Used for keeping the form elements on the DOM until user confirms to
/// save or cancel them.
///
/// See: https://github.com/flutter/flutter/blob/bf9f3a3dcfea3022f9cf2dfc3ab10b120b48b19d/packages/flutter/lib/src/services/text_input.dart#L1277
final Map<String, DomHTMLFormElement> formsOnTheDom = <String, DomHTMLFormElement>{};

/// Should be used as a singleton to provide support for text editing in
/// Flutter Web.
///
/// The approach is "hybrid" because it relies on Flutter for
/// displaying, and HTML for user interactions:
///
/// - HTML's contentEditable feature handles typing and text changes.
/// - HTML's selection API handles selection changes and cursor movements.
class HybridTextEditing {
  /// Private constructor so this class can be a singleton.
  ///
  /// The constructor also decides which text editing strategy to use depending
  /// on the operating system and browser engine.
  HybridTextEditing();

  late final TextEditingChannel channel = TextEditingChannel(this);

  /// A CSS class name used to identify all elements used for text editing.
  @visibleForTesting
  static const String textEditingClass = 'flt-text-editing';

  int? _clientId;

  /// Flag which shows if there is an ongoing editing.
  ///
  /// Also used to define if a keyboard is needed.
  bool isEditing = false;

  InputConfiguration? configuration;

  DefaultTextEditingStrategy? debugTextEditingStrategyOverride;

  /// Supplies the DOM element used for editing.
  late final DefaultTextEditingStrategy strategy =
      debugTextEditingStrategyOverride ??
      (EngineSemantics.instance.semanticsEnabled
          ? SemanticsTextEditingStrategy.ensureInitialized(this)
          : createDefaultTextEditingStrategy(this));

  void acceptCommand(TextInputCommand command, ui.VoidCallback callback) {
    if (_debugPrintTextInputCommands) {
      print('flutter/textinput channel command: ${command.runtimeType}');
    }
    command.run(this);
    callback();
  }

  void _startEditing() {
    assert(!isEditing);
    isEditing = true;
    strategy.enable(
      configuration!,
      onChange: (EditingState? editingState, TextEditingDeltaState? editingDeltaState) {
        if (configuration!.enableDeltaModel) {
          channel.updateEditingStateWithDelta(_clientId, editingDeltaState);
        } else {
          channel.updateEditingState(_clientId, editingState);
        }
      },
      onAction: (String? inputAction) {
        channel.performAction(_clientId, inputAction);
      },
    );
  }

  void stopEditing() {
    assert(isEditing);
    isEditing = false;
    strategy.disable();
  }

  void sendTextConnectionClosedToFrameworkIfAny() {
    if (isEditing) {
      stopEditing();
      channel.onConnectionClosed(_clientId);
    }
  }
}

/// Information on the font and alignment of a text editing element.
///
/// This information is received via TextInput.setStyle message.
class EditableTextStyle {
  EditableTextStyle({
    required this.textDirection,
    required this.fontSize,
    required this.textAlign,
    required this.fontFamily,
    required this.fontWeight,
  });

  factory EditableTextStyle.fromFrameworkMessage(Map<String, dynamic> flutterStyle) {
    assert(flutterStyle.containsKey('fontSize'));
    assert(flutterStyle.containsKey('fontFamily'));
    assert(flutterStyle.containsKey('textAlignIndex'));
    assert(flutterStyle.containsKey('textDirectionIndex'));

    final int textAlignIndex = flutterStyle['textAlignIndex'] as int;
    final int textDirectionIndex = flutterStyle['textDirectionIndex'] as int;
    final int? fontWeightIndex = flutterStyle['fontWeightIndex'] as int?;

    // Convert [fontWeightIndex] to its CSS equivalent value.
    final String fontWeight =
        fontWeightIndex != null ? fontWeightIndexToCss(fontWeightIndex: fontWeightIndex) : 'normal';

    // Also convert [textAlignIndex] and [textDirectionIndex] to their
    // corresponding enum values in [ui.TextAlign] and [ui.TextDirection]
    // respectively.
    return EditableTextStyle(
      fontSize: flutterStyle.tryDouble('fontSize'),
      fontFamily: flutterStyle.tryString('fontFamily'),
      textAlign: ui.TextAlign.values[textAlignIndex],
      textDirection: ui.TextDirection.values[textDirectionIndex],
      fontWeight: fontWeight,
    );
  }

  /// This information will be used for changing the style of the hidden input
  /// element, which will match it's size to the size of the editable widget.
  final double? fontSize;
  final String fontWeight;
  final String? fontFamily;
  final ui.TextAlign textAlign;
  final ui.TextDirection textDirection;

  String? get align => textAlignToCssValue(textAlign, textDirection);

  String get cssFont => '$fontWeight ${fontSize}px ${canonicalizeFontFamily(fontFamily)}';

  void applyToDomElement(DomHTMLElement domElement) {
    domElement.style
      ..textAlign = align!
      ..font = cssFont;
  }
}

/// Describes the location and size of the editing element on the screen.
///
/// This information is received via "TextInput.setEditableSizeAndTransform"
/// message from the framework.
@immutable
class EditableTextGeometry {
  const EditableTextGeometry({
    required this.width,
    required this.height,
    required this.globalTransform,
  });

  /// Parses the geometry from a message sent by the framework.
  factory EditableTextGeometry.fromFrameworkMessage(Map<String, dynamic> encodedGeometry) {
    assert(encodedGeometry.containsKey('width'));
    assert(encodedGeometry.containsKey('height'));
    assert(encodedGeometry.containsKey('transform'));

    final List<double> transformList = List<double>.from(
      encodedGeometry.readList('transform').map((final dynamic e) => (e as num).toDouble()),
    );
    return EditableTextGeometry(
      width: encodedGeometry.readDouble('width'),
      height: encodedGeometry.readDouble('height'),
      globalTransform: Float32List.fromList(transformList),
    );
  }

  /// The width of the editable in local coordinates, i.e. before applying [globalTransform].
  final double width;

  /// The height of the editable in local coordinates, i.e. before applying [globalTransform].
  final double height;

  /// The aggregate transform rooted at the global (screen) coordinate system
  /// that places and sizes the editable.
  ///
  /// For correct sizing this transform must be applied to the [width] and
  /// [height] fields.
  final Float32List globalTransform;

  /// Applies this geometry to the DOM element.
  ///
  /// This assumes that the parent of the [domElement] has identity transform
  /// applied to it (i.e. the default). If the parent has a non-identity
  /// transform applied, this method will misplace the [domElement]. For
  /// example, if the editable DOM element is nested inside the semantics
  /// tree the semantics tree provides the placement parameters, in which
  /// case this method should not be used.
  void applyToDomElement(DomHTMLElement domElement) {
    final String cssTransform = float64ListToCssTransform(globalTransform);
    domElement.style
      ..width = '${width}px'
      ..height = '${height}px'
      ..transform = cssTransform;
  }
}
