// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/rendering.dart';
///
/// @docImport 'live_text.dart';
/// @docImport 'scribe.dart';
/// @docImport 'text_formatter.dart';
library;

import 'dart:async';
import 'dart:io' show Platform;
import 'dart:ui' show FlutterView, FontWeight, Offset, Rect, Size, TextAlign, TextDirection;

import 'package:flutter/foundation.dart';
import 'package:vector_math/vector_math_64.dart' show Matrix4;

import 'autofill.dart';
import 'binding.dart';
import 'clipboard.dart' show Clipboard;
import 'keyboard_inserted_content.dart';
import 'message_codec.dart';
import 'platform_channel.dart';
import 'system_channels.dart';
import 'text_editing.dart';
import 'text_editing_delta.dart';

export 'dart:ui'
    show
        Brightness,
        FontWeight,
        Offset,
        Rect,
        Size,
        TextAlign,
        TextDirection,
        TextPosition,
        TextRange;

export 'package:vector_math/vector_math_64.dart' show Matrix4;

export 'autofill.dart' show AutofillConfiguration, AutofillScope;
export 'text_editing.dart' show TextSelection;

// TODO(a14n): the following export leads to Segmentation fault, see https://github.com/flutter/flutter/issues/106332
// export 'text_editing_delta.dart' show TextEditingDelta;

/// Indicates how to handle the intelligent replacement of dashes in text input.
///
/// See also:
///
///  * [TextField.smartDashesType]
///  * [CupertinoTextField.smartDashesType]
///  * [EditableText.smartDashesType]
///  * [SmartQuotesType]
///  * <https://developer.apple.com/documentation/uikit/uitextinputtraits>
enum SmartDashesType {
  /// Smart dashes is disabled.
  ///
  /// This corresponds to the
  /// ["no" value of UITextSmartDashesType](https://developer.apple.com/documentation/uikit/uitextsmartdashestype/no).
  disabled,

  /// Smart dashes is enabled.
  ///
  /// This corresponds to the
  /// ["yes" value of UITextSmartDashesType](https://developer.apple.com/documentation/uikit/uitextsmartdashestype/yes).
  enabled,
}

/// Indicates how to handle the intelligent replacement of quotes in text input.
///
/// See also:
///
///  * [TextField.smartQuotesType]
///  * [CupertinoTextField.smartQuotesType]
///  * [EditableText.smartQuotesType]
///  * <https://developer.apple.com/documentation/uikit/uitextinputtraits>
enum SmartQuotesType {
  /// Smart quotes is disabled.
  ///
  /// This corresponds to the
  /// ["no" value of UITextSmartQuotesType](https://developer.apple.com/documentation/uikit/uitextsmartquotestype/no).
  disabled,

  /// Smart quotes is enabled.
  ///
  /// This corresponds to the
  /// ["yes" value of UITextSmartQuotesType](https://developer.apple.com/documentation/uikit/uitextsmartquotestype/yes).
  enabled,
}

/// The type of information for which to optimize the text input control.
///
/// On Android, behavior may vary across device and keyboard provider.
///
/// This class stays as close to `Enum` interface as possible, and allows
/// for additional flags for some input types. For example, numeric input
/// can specify whether it supports decimal numbers and/or signed numbers.
@immutable
class TextInputType {
  const TextInputType._(this.index) : signed = null, decimal = null;

  /// Optimize for numerical information.
  ///
  /// Requests a numeric keyboard with additional settings.
  /// The [signed] and [decimal] parameters are optional.
  const TextInputType.numberWithOptions({this.signed = false, this.decimal = false}) : index = 2;

  /// Enum value index, corresponds to one of the [values].
  final int index;

  /// The number is signed, allowing a positive or negative sign at the start.
  ///
  /// This flag is only used for the [number] input type, otherwise `null`.
  /// Use `const TextInputType.numberWithOptions(signed: true)` to set this.
  final bool? signed;

  /// The number is decimal, allowing a decimal point to provide fractional.
  ///
  /// This flag is only used for the [number] input type, otherwise `null`.
  /// Use `const TextInputType.numberWithOptions(decimal: true)` to set this.
  final bool? decimal;

  /// Optimize for textual information.
  ///
  /// Requests the default platform keyboard.
  static const TextInputType text = TextInputType._(0);

  /// Optimize for multiline textual information.
  ///
  /// Requests the default platform keyboard, but accepts newlines when the
  /// enter key is pressed. This is the input type used for all multiline text
  /// fields.
  static const TextInputType multiline = TextInputType._(1);

  /// Optimize for unsigned numerical information without a decimal point.
  ///
  /// Requests a default keyboard with ready access to the number keys.
  /// Additional options, such as decimal point and/or positive/negative
  /// signs, can be requested using [TextInputType.numberWithOptions].
  static const TextInputType number = TextInputType.numberWithOptions();

  /// Optimize for telephone numbers.
  ///
  /// Requests a keyboard with ready access to the number keys, "*", and "#".
  static const TextInputType phone = TextInputType._(3);

  /// Optimize for date and time information.
  ///
  /// On iOS, requests the default keyboard.
  ///
  /// On Android, requests a keyboard with ready access to the number keys,
  /// ":", and "-".
  static const TextInputType datetime = TextInputType._(4);

  /// Optimize for email addresses.
  ///
  /// Requests a keyboard with ready access to the "@" and "." keys.
  static const TextInputType emailAddress = TextInputType._(5);

  /// Optimize for URLs.
  ///
  /// Requests a keyboard with ready access to the "/" and "." keys.
  static const TextInputType url = TextInputType._(6);

  /// Optimize for passwords that are visible to the user.
  ///
  /// Requests a keyboard with ready access to both letters and numbers.
  static const TextInputType visiblePassword = TextInputType._(7);

  /// Optimized for a person's name.
  ///
  /// On iOS, requests the
  /// [UIKeyboardType.namePhonePad](https://developer.apple.com/documentation/uikit/uikeyboardtype/namephonepad)
  /// keyboard, a keyboard optimized for entering a person’s name or phone number.
  /// Does not support auto-capitalization.
  ///
  /// On Android, requests a keyboard optimized for
  /// [TYPE_TEXT_VARIATION_PERSON_NAME](https://developer.android.com/reference/android/text/InputType#TYPE_TEXT_VARIATION_PERSON_NAME).
  static const TextInputType name = TextInputType._(8);

  /// Optimized for postal mailing addresses.
  ///
  /// On iOS, requests the default keyboard.
  ///
  /// On Android, requests a keyboard optimized for
  /// [TYPE_TEXT_VARIATION_POSTAL_ADDRESS](https://developer.android.com/reference/android/text/InputType#TYPE_TEXT_VARIATION_POSTAL_ADDRESS).
  static const TextInputType streetAddress = TextInputType._(9);

  /// Prevent the OS from showing the on-screen virtual keyboard.
  static const TextInputType none = TextInputType._(10);

  /// Optimized for web searches.
  ///
  /// Requests a keyboard that includes keys useful for web searches as well as URLs.
  ///
  /// On iOS, requests a default keyboard with ready access to the "." key. In contrast to
  /// [url], a space bar is available.
  ///
  /// On Android this is remapped to the [url] keyboard type as it always shows a space bar.
  static const TextInputType webSearch = TextInputType._(11);

  /// Optimized for social media.
  ///
  /// Requests a keyboard that includes keys useful for handles and tags.
  ///
  /// On iOS, requests a default keyboard with ready access to the "@" and "#" keys.
  ///
  /// On Android this is remapped to the [emailAddress] keyboard type as it always shows the "@" key.
  static const TextInputType twitter = TextInputType._(12);

  /// All possible enum values.
  static const List<TextInputType> values = <TextInputType>[
    text,
    multiline,
    number,
    phone,
    datetime,
    emailAddress,
    url,
    visiblePassword,
    name,
    streetAddress,
    none,
    webSearch,
    twitter,
  ];

  // Corresponding string name for each of the [values].
  static const List<String> _names = <String>[
    'text',
    'multiline',
    'number',
    'phone',
    'datetime',
    'emailAddress',
    'url',
    'visiblePassword',
    'name',
    'address',
    'none',
    'webSearch',
    'twitter',
  ];

  // Enum value name, this is what enum.toString() would normally return.
  String get _name => 'TextInputType.${_names[index]}';

  /// Returns a representation of this object as a JSON object.
  Map<String, dynamic> toJson() {
    return <String, dynamic>{'name': _name, 'signed': signed, 'decimal': decimal};
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'TextInputType')}('
        'name: $_name, '
        'signed: $signed, '
        'decimal: $decimal)';
  }

  @override
  bool operator ==(Object other) {
    return other is TextInputType &&
        other.index == index &&
        other.signed == signed &&
        other.decimal == decimal;
  }

  @override
  int get hashCode => Object.hash(index, signed, decimal);
}

/// An action the user has requested the text input control to perform.
///
/// Each action represents a logical meaning, and also configures the soft
/// keyboard to display a certain kind of action button. The visual appearance
/// of the action button might differ between versions of the same OS.
///
/// Despite the logical meaning of each action, choosing a particular
/// [TextInputAction] does not necessarily cause any specific behavior to
/// happen, other than changing the focus when appropriate. It is up to the
/// developer to ensure that the behavior that occurs when an action button is
/// pressed is appropriate for the action button chosen.
///
/// For example: If the user presses the keyboard action button on iOS when it
/// reads "Emergency Call", the result should not be a focus change to the next
/// TextField. This behavior is not logically appropriate for a button that says
/// "Emergency Call".
///
/// See [EditableText] for more information about customizing action button
/// behavior.
///
/// Most [TextInputAction]s are supported equally by both Android and iOS.
/// However, there is not a complete, direct mapping between Android's IME input
/// types and iOS's keyboard return types. Therefore, some [TextInputAction]s
/// are inappropriate for one of the platforms. If a developer chooses an
/// inappropriate [TextInputAction] when running in debug mode, an error will be
/// thrown. If the same thing is done in release mode, then instead of sending
/// the inappropriate value, Android will use "unspecified" on the platform
/// side and iOS will use "default" on the platform side.
///
/// See also:
///
///  * [TextInput], which configures the platform's keyboard setup.
///  * [EditableText], which invokes callbacks when the action button is pressed.
//
// This class has been cloned to `flutter_driver/lib/src/common/action.dart` as `TextInputAction`,
// and must be kept in sync.
enum TextInputAction {
  /// Logical meaning: There is no relevant input action for the current input
  /// source, e.g., [TextField].
  ///
  /// Android: Corresponds to Android's "IME_ACTION_NONE". The keyboard setup
  /// is decided by the OS. The keyboard will likely show a return key.
  ///
  /// iOS: iOS does not have a keyboard return type of "none." It is
  /// inappropriate to choose this [TextInputAction] when running on iOS.
  none,

  /// Logical meaning: Let the OS decide which action is most appropriate.
  ///
  /// Android: Corresponds to Android's "IME_ACTION_UNSPECIFIED". The OS chooses
  /// which keyboard action to display. The decision will likely be a done
  /// button or a return key.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyDefault". The title displayed in
  /// the action button is "return".
  unspecified,

  /// Logical meaning: The user is done providing input to a group of inputs
  /// (like a form). Some kind of finalization behavior should now take place.
  ///
  /// Android: Corresponds to Android's "IME_ACTION_DONE". The OS displays a
  /// button that represents completion, e.g., a checkmark button.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyDone". The title displayed in the
  /// action button is "Done".
  done,

  /// Logical meaning: The user has entered some text that represents a
  /// destination, e.g., a restaurant name. The "go" button is intended to take
  /// the user to a part of the app that corresponds to this destination.
  ///
  /// Android: Corresponds to Android's "IME_ACTION_GO". The OS displays a
  /// button that represents taking "the user to the target of the text they
  /// typed", e.g., a right-facing arrow button.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyGo". The title displayed in the
  /// action button is "Go".
  go,

  /// Logical meaning: Execute a search query.
  ///
  /// Android: Corresponds to Android's "IME_ACTION_SEARCH". The OS displays a
  /// button that represents a search, e.g., a magnifying glass button.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeySearch". The title displayed in the
  /// action button is "Search".
  search,

  /// Logical meaning: Sends something that the user has composed, e.g., an
  /// email or a text message.
  ///
  /// Android: Corresponds to Android's "IME_ACTION_SEND". The OS displays a
  /// button that represents sending something, e.g., a paper plane button.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeySend". The title displayed in the
  /// action button is "Send".
  send,

  /// Logical meaning: The user is done with the current input source and wants
  /// to move to the next one.
  ///
  /// Moves the focus to the next focusable item in the same [FocusScope].
  ///
  /// Android: Corresponds to Android's "IME_ACTION_NEXT". The OS displays a
  /// button that represents moving forward, e.g., a right-facing arrow button.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyNext". The title displayed in the
  /// action button is "Next".
  next,

  /// Logical meaning: The user wishes to return to the previous input source
  /// in the group, e.g., a form with multiple [TextField]s.
  ///
  /// Moves the focus to the previous focusable item in the same [FocusScope].
  ///
  /// Android: Corresponds to Android's "IME_ACTION_PREVIOUS". The OS displays a
  /// button that represents moving backward, e.g., a left-facing arrow button.
  ///
  /// iOS: iOS does not have a keyboard return type of "previous." It is
  /// inappropriate to choose this [TextInputAction] when running on iOS.
  previous,

  /// Logical meaning: In iOS apps, it is common for a "Back" button and
  /// "Continue" button to appear at the top of the screen. However, when the
  /// keyboard is open, these buttons are often hidden off-screen. Therefore,
  /// the purpose of the "Continue" return key on iOS is to make the "Continue"
  /// button available when the user is entering text.
  ///
  /// Historical context aside, [TextInputAction.continueAction] can be used any
  /// time that the term "Continue" seems most appropriate for the given action.
  ///
  /// Android: Android does not have an IME input type of "continue." It is
  /// inappropriate to choose this [TextInputAction] when running on Android.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyContinue". The title displayed in the
  /// action button is "Continue". This action is only available on iOS 9.0+.
  ///
  /// The reason that this value has "Action" post-fixed to it is because
  /// "continue" is a reserved word in Dart, as well as many other languages.
  continueAction,

  /// Logical meaning: The user wants to join something, e.g., a wireless
  /// network.
  ///
  /// Android: Android does not have an IME input type of "join." It is
  /// inappropriate to choose this [TextInputAction] when running on Android.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyJoin". The title displayed in the
  /// action button is "Join".
  join,

  /// Logical meaning: The user wants routing options, e.g., driving directions.
  ///
  /// Android: Android does not have an IME input type of "route." It is
  /// inappropriate to choose this [TextInputAction] when running on Android.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyRoute". The title displayed in the
  /// action button is "Route".
  route,

  /// Logical meaning: Initiate a call to emergency services.
  ///
  /// Android: Android does not have an IME input type of "emergencyCall." It is
  /// inappropriate to choose this [TextInputAction] when running on Android.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyEmergencyCall". The title displayed
  /// in the action button is "Emergency Call".
  emergencyCall,

  /// Logical meaning: Insert a newline character in the focused text input,
  /// e.g., [TextField].
  ///
  /// Android: Corresponds to Android's "IME_ACTION_NONE". The OS displays a
  /// button that represents a new line, e.g., a carriage return button.
  ///
  /// iOS: Corresponds to iOS's "UIReturnKeyDefault". The title displayed in the
  /// action button is "return".
  ///
  /// The term [TextInputAction.newline] exists in Flutter but not in Android
  /// or iOS. The reason for introducing this term is so that developers can
  /// achieve the common result of inserting new lines without needing to
  /// understand the various IME actions on Android and return keys on iOS.
  /// Thus, [TextInputAction.newline] is a convenience term that alleviates the
  /// need to understand the underlying platforms to achieve this common behavior.
  newline,
}

/// Configures how the platform keyboard will select an uppercase or
/// lowercase keyboard.
///
/// Only supports text keyboards, other keyboard types will ignore this
/// configuration. Capitalization is locale-aware.
enum TextCapitalization {
  /// Defaults to an uppercase keyboard for the first letter of each word.
  ///
  /// Corresponds to `InputType.TYPE_TEXT_FLAG_CAP_WORDS` on Android, and
  /// `UITextAutocapitalizationTypeWords` on iOS.
  words,

  /// Defaults to an uppercase keyboard for the first letter of each sentence.
  ///
  /// Corresponds to `InputType.TYPE_TEXT_FLAG_CAP_SENTENCES` on Android, and
  /// `UITextAutocapitalizationTypeSentences` on iOS.
  sentences,

  /// Defaults to an uppercase keyboard for each character.
  ///
  /// Corresponds to `InputType.TYPE_TEXT_FLAG_CAP_CHARACTERS` on Android, and
  /// `UITextAutocapitalizationTypeAllCharacters` on iOS.
  characters,

  /// Defaults to a lowercase keyboard.
  none,
}

/// Controls the visual appearance of the text input control.
///
/// Many [TextInputAction]s are common between Android and iOS. However, if an
/// [inputAction] is provided that is not supported by the current
/// platform in debug mode, an error will be thrown when the corresponding
/// text input is attached. For example, providing iOS's "emergencyCall"
/// action when running on an Android device will result in an error when in
/// debug mode. In release mode, incompatible [TextInputAction]s are replaced
/// either with "unspecified" on Android, or "default" on iOS. Appropriate
/// [inputAction]s can be chosen by checking the current platform and then
/// selecting the appropriate action.
///
/// See also:
///
///  * [TextInput.attach]
///  * [TextInputAction]
@immutable
class TextInputConfiguration {
  /// Creates configuration information for a text input control.
  ///
  /// All arguments have default values, except [actionLabel]. Only
  /// [actionLabel] may be null.
  const TextInputConfiguration({
    this.viewId,
    this.inputType = TextInputType.text,
    this.readOnly = false,
    this.obscureText = false,
    this.autocorrect = true,
    SmartDashesType? smartDashesType,
    SmartQuotesType? smartQuotesType,
    this.enableSuggestions = true,
    this.enableInteractiveSelection = true,
    this.actionLabel,
    this.inputAction = TextInputAction.done,
    this.keyboardAppearance = Brightness.light,
    this.textCapitalization = TextCapitalization.none,
    this.autofillConfiguration = AutofillConfiguration.disabled,
    this.enableIMEPersonalizedLearning = true,
    this.allowedMimeTypes = const <String>[],
    this.enableDeltaModel = false,
  }) : smartDashesType =
           smartDashesType ?? (obscureText ? SmartDashesType.disabled : SmartDashesType.enabled),
       smartQuotesType =
           smartQuotesType ?? (obscureText ? SmartQuotesType.disabled : SmartQuotesType.enabled);

  /// The ID of the view that the text input belongs to.
  ///
  /// See also:
  ///
  /// * [FlutterView], which is the view that the ID points to.
  /// * [View], which is a widget that wraps a [FlutterView].
  final int? viewId;

  /// The type of information for which to optimize the text input control.
  final TextInputType inputType;

  /// Whether the text field can be edited or not.
  ///
  /// Defaults to false.
  final bool readOnly;

  /// Whether to hide the text being edited (e.g., for passwords).
  ///
  /// Defaults to false.
  final bool obscureText;

  /// Whether to enable autocorrection.
  ///
  /// Defaults to true.
  final bool autocorrect;

  /// The configuration to use for autofill.
  ///
  /// Defaults to null, in which case no autofill information will be provided
  /// to the platform. This will prevent the corresponding input field from
  /// participating in autofills triggered by other fields. Additionally, on
  /// Android and web, setting [autofillConfiguration] to null disables autofill.
  final AutofillConfiguration autofillConfiguration;

  /// {@template flutter.services.TextInputConfiguration.smartDashesType}
  /// Whether to allow the platform to automatically format dashes.
  ///
  /// This flag only affects iOS versions 11 and above. It sets
  /// [`UITextSmartDashesType`](https://developer.apple.com/documentation/uikit/uitextsmartdashestype?language=objc)
  /// in the engine. When true, it passes
  /// [`UITextSmartDashesTypeYes`](https://developer.apple.com/documentation/uikit/uitextsmartdashestype/uitextsmartdashestypeyes?language=objc),
  /// and when false, it passes
  /// [`UITextSmartDashesTypeNo`](https://developer.apple.com/documentation/uikit/uitextsmartdashestype/uitextsmartdashestypeno?language=objc).
  ///
  /// As an example of what this does, two consecutive hyphen characters will be
  /// automatically replaced with one en dash, and three consecutive hyphens
  /// will become one em dash.
  ///
  /// Defaults to true, unless [obscureText] is true, when it defaults to false.
  /// This is to avoid the problem where password fields receive autoformatted
  /// characters.
  ///
  /// See also:
  ///
  ///  * [smartQuotesType]
  ///  * <https://developer.apple.com/documentation/uikit/uitextinputtraits>
  /// {@endtemplate}
  final SmartDashesType smartDashesType;

  /// {@template flutter.services.TextInputConfiguration.smartQuotesType}
  /// Whether to allow the platform to automatically format quotes.
  ///
  /// This flag only affects iOS. It sets
  /// [`UITextSmartQuotesType`](https://developer.apple.com/documentation/uikit/uitextsmartquotestype?language=objc)
  /// in the engine. When true, it passes
  /// [`UITextSmartQuotesTypeYes`](https://developer.apple.com/documentation/uikit/uitextsmartquotestype/uitextsmartquotestypeyes?language=objc),
  /// and when false, it passes
  /// [`UITextSmartQuotesTypeNo`](https://developer.apple.com/documentation/uikit/uitextsmartquotestype/uitextsmartquotestypeno?language=objc).
  ///
  /// As an example of what this does, a standard vertical double quote
  /// character will be automatically replaced by a left or right double quote
  /// depending on its position in a word.
  ///
  /// Defaults to true, unless [obscureText] is true, when it defaults to false.
  /// This is to avoid the problem where password fields receive autoformatted
  /// characters.
  ///
  /// See also:
  ///
  ///  * [smartDashesType]
  ///  * <https://developer.apple.com/documentation/uikit/uitextinputtraits>
  /// {@endtemplate}
  final SmartQuotesType smartQuotesType;

  /// {@template flutter.services.TextInputConfiguration.enableSuggestions}
  /// Whether to show input suggestions as the user types.
  ///
  /// This flag only affects Android. On iOS, suggestions are tied directly to
  /// [autocorrect], so that suggestions are only shown when [autocorrect] is
  /// true. On Android autocorrection and suggestion are controlled separately.
  ///
  /// Defaults to true.
  ///
  /// See also:
  ///
  ///  * <https://developer.android.com/reference/android/text/InputType.html#TYPE_TEXT_FLAG_NO_SUGGESTIONS>
  /// {@endtemplate}
  final bool enableSuggestions;

  /// Whether a user can change its selection.
  ///
  /// This flag only affects iOS VoiceOver. On Android Talkback, the selection
  /// change is sent through semantics actions and is directly disabled from
  /// the widget side.
  ///
  /// Defaults to true.
  final bool enableInteractiveSelection;

  /// What text to display in the text input control's action button.
  final String? actionLabel;

  /// What kind of action to request for the action button on the IME.
  final TextInputAction inputAction;

  /// Specifies how platforms may automatically capitalize text entered by the
  /// user.
  ///
  /// Defaults to [TextCapitalization.none].
  ///
  /// See also:
  ///
  ///  * [TextCapitalization], for a description of each capitalization behavior.
  final TextCapitalization textCapitalization;

  /// The appearance of the keyboard.
  ///
  /// This setting is only honored on iOS devices.
  ///
  /// Defaults to [Brightness.light].
  final Brightness keyboardAppearance;

  /// {@template flutter.services.TextInputConfiguration.enableIMEPersonalizedLearning}
  /// Whether to enable that the IME update personalized data such as typing
  /// history and user dictionary data.
  ///
  /// This flag only affects Android. On iOS, there is no equivalent flag.
  ///
  /// Defaults to true.
  ///
  /// See also:
  ///
  ///  * <https://developer.android.com/reference/android/view/inputmethod/EditorInfo#IME_FLAG_NO_PERSONALIZED_LEARNING>
  /// {@endtemplate}
  final bool enableIMEPersonalizedLearning;

  /// {@macro flutter.widgets.contentInsertionConfiguration.allowedMimeTypes}
  final List<String> allowedMimeTypes;

  /// Creates a copy of this [TextInputConfiguration] with the given fields
  /// replaced with new values.
  TextInputConfiguration copyWith({
    int? viewId,
    TextInputType? inputType,
    bool? readOnly,
    bool? obscureText,
    bool? autocorrect,
    SmartDashesType? smartDashesType,
    SmartQuotesType? smartQuotesType,
    bool? enableSuggestions,
    bool? enableInteractiveSelection,
    String? actionLabel,
    TextInputAction? inputAction,
    Brightness? keyboardAppearance,
    TextCapitalization? textCapitalization,
    bool? enableIMEPersonalizedLearning,
    List<String>? allowedMimeTypes,
    AutofillConfiguration? autofillConfiguration,
    bool? enableDeltaModel,
  }) {
    return TextInputConfiguration(
      viewId: viewId ?? this.viewId,
      inputType: inputType ?? this.inputType,
      readOnly: readOnly ?? this.readOnly,
      obscureText: obscureText ?? this.obscureText,
      autocorrect: autocorrect ?? this.autocorrect,
      smartDashesType: smartDashesType ?? this.smartDashesType,
      smartQuotesType: smartQuotesType ?? this.smartQuotesType,
      enableSuggestions: enableSuggestions ?? this.enableSuggestions,
      enableInteractiveSelection: enableInteractiveSelection ?? this.enableInteractiveSelection,
      actionLabel: actionLabel ?? this.actionLabel,
      inputAction: inputAction ?? this.inputAction,
      textCapitalization: textCapitalization ?? this.textCapitalization,
      keyboardAppearance: keyboardAppearance ?? this.keyboardAppearance,
      enableIMEPersonalizedLearning:
          enableIMEPersonalizedLearning ?? this.enableIMEPersonalizedLearning,
      allowedMimeTypes: allowedMimeTypes ?? this.allowedMimeTypes,
      autofillConfiguration: autofillConfiguration ?? this.autofillConfiguration,
      enableDeltaModel: enableDeltaModel ?? this.enableDeltaModel,
    );
  }

  /// Whether to enable that the engine sends text input updates to the
  /// framework as [TextEditingDelta]'s or as one [TextEditingValue].
  ///
  /// Enabling this flag results in granular text updates being received from the
  /// platform's text input control.
  ///
  /// When this is enabled:
  ///  * You must implement [DeltaTextInputClient] and not [TextInputClient] to
  ///    receive granular updates from the platform's text input.
  ///  * Platform text input updates will come through
  ///    [DeltaTextInputClient.updateEditingValueWithDeltas].
  ///  * If [TextInputClient] is implemented with this property enabled then
  ///    you will experience unexpected behavior as [TextInputClient] does not implement
  ///    a delta channel.
  ///
  /// When this is disabled:
  ///  * If [DeltaTextInputClient] is implemented then updates for the
  ///    editing state will continue to come through the
  ///    [DeltaTextInputClient.updateEditingValue] channel.
  ///  * If [TextInputClient] is implemented then updates for the editing
  ///    state will come through [TextInputClient.updateEditingValue].
  ///
  /// Defaults to false.
  final bool enableDeltaModel;

  /// Returns a representation of this object as a JSON object.
  Map<String, dynamic> toJson() {
    final Map<String, dynamic>? autofill = autofillConfiguration.toJson();
    return <String, dynamic>{
      'viewId': viewId,
      'inputType': inputType.toJson(),
      'readOnly': readOnly,
      'obscureText': obscureText,
      'autocorrect': autocorrect,
      'smartDashesType': smartDashesType.index.toString(),
      'smartQuotesType': smartQuotesType.index.toString(),
      'enableSuggestions': enableSuggestions,
      'enableInteractiveSelection': enableInteractiveSelection,
      'actionLabel': actionLabel,
      'inputAction': inputAction.toString(),
      'textCapitalization': textCapitalization.toString(),
      'keyboardAppearance': keyboardAppearance.toString(),
      'enableIMEPersonalizedLearning': enableIMEPersonalizedLearning,
      'contentCommitMimeTypes': allowedMimeTypes,
      if (autofill != null) 'autofill': autofill,
      'enableDeltaModel': enableDeltaModel,
    };
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is TextInputConfiguration &&
        other.viewId == viewId &&
        other.inputType == inputType &&
        other.readOnly == readOnly &&
        other.obscureText == obscureText &&
        other.autocorrect == autocorrect &&
        other.smartDashesType == smartDashesType &&
        other.smartQuotesType == smartQuotesType &&
        other.enableSuggestions == enableSuggestions &&
        other.enableInteractiveSelection == enableInteractiveSelection &&
        other.actionLabel == actionLabel &&
        other.inputAction == inputAction &&
        other.keyboardAppearance == keyboardAppearance &&
        other.textCapitalization == textCapitalization &&
        other.autofillConfiguration == autofillConfiguration &&
        other.enableIMEPersonalizedLearning == enableIMEPersonalizedLearning &&
        listEquals(other.allowedMimeTypes, allowedMimeTypes) &&
        other.enableDeltaModel == enableDeltaModel;
  }

  @override
  int get hashCode {
    return Object.hash(
      viewId,
      inputType,
      readOnly,
      obscureText,
      autocorrect,
      smartDashesType,
      smartQuotesType,
      enableSuggestions,
      enableInteractiveSelection,
      actionLabel,
      inputAction,
      keyboardAppearance,
      textCapitalization,
      autofillConfiguration,
      enableIMEPersonalizedLearning,
      Object.hashAll(allowedMimeTypes),
      enableDeltaModel,
    );
  }

  @override
  String toString() {
    final List<String> description = <String>[
      if (viewId != null) 'viewId: $viewId',
      'inputType: $inputType',
      'readOnly: $readOnly',
      'obscureText: $obscureText',
      'autocorrect: $autocorrect',
      'smartDashesType: $smartDashesType',
      'smartQuotesType: $smartQuotesType',
      'enableSuggestions: $enableSuggestions',
      'enableInteractiveSelection: $enableInteractiveSelection',
      if (actionLabel != null) 'actionLabel: $actionLabel',
      'inputAction: $inputAction',
      'keyboardAppearance: $keyboardAppearance',
      'textCapitalization: $textCapitalization',
      'autofillConfiguration: $autofillConfiguration',
      'enableIMEPersonalizedLearning: $enableIMEPersonalizedLearning',
      'allowedMimeTypes: $allowedMimeTypes',
      'enableDeltaModel: $enableDeltaModel',
    ];
    return 'TextInputConfiguration(${description.join(', ')})';
  }
}

TextAffinity? _toTextAffinity(String? affinity) {
  return switch (affinity) {
    'TextAffinity.downstream' => TextAffinity.downstream,
    'TextAffinity.upstream' => TextAffinity.upstream,
    _ => null,
  };
}

/// The state of a "floating cursor" drag on an iOS soft keyboard.
///
/// The "floating cursor" cursor-positioning mode is an iOS feature used to
/// precisely position the caret in some editable text using certain touch
/// gestures. As an example, when the user long-presses the spacebar on the iOS
/// virtual keyboard, iOS enters floating cursor mode where the whole keyboard
/// becomes a trackpad. In this mode, there are two visible cursors. One, the
/// floating cursor, hovers over the text, following the user's horizontal
/// movements exactly and snapping to lines vertically. The other, the
/// placeholder cursor, is a "shadow" that also snaps to the actual location
/// where the cursor will go horizontally when the user releases the trackpad.
///
/// The floating cursor renders over the text field, while the placeholder
/// cursor is a faint shadow of the cursor rendered in the text field in the
/// location between characters where the cursor will drop into when released.
/// The placeholder cursor is a faint vertical bar, while the floating cursor
/// has the same appearance as a normal cursor (a blue vertical bar).
///
/// This feature works out-of-the-box with Flutter. Support is built into
/// [EditableText].
///
/// See also:
///
///  * [EditableText.backgroundCursorColor], which configures the color of the
///    placeholder cursor while the floating cursor is being dragged.
enum FloatingCursorDragState {
  /// A user has just activated a floating cursor by long pressing on the
  /// spacebar.
  Start,

  /// A user is dragging a floating cursor.
  Update,

  /// A user has lifted their finger off the screen after using a floating
  /// cursor.
  End,
}

/// The current state and position of the floating cursor.
///
/// See also:
///
///  * [FloatingCursorDragState], which explains the floating cursor feature in
///    detail.
class RawFloatingCursorPoint {
  /// Creates information for setting the position and state of a floating
  /// cursor.
  ///
  /// [state] must not be null and [offset] must not be null if the state is
  /// [FloatingCursorDragState.Update].
  RawFloatingCursorPoint({this.offset, this.startLocation, required this.state})
    : assert(state != FloatingCursorDragState.Update || offset != null);

  /// The raw position of the floating cursor as determined by the iOS sdk.
  final Offset? offset;

  /// Represents the starting location when initiating a floating cursor via long press.
  /// This is a tuple where the first item is the local offset and the second item is the new caret position.
  /// This is only non-null when a floating cursor is started.
  final (Offset, TextPosition)? startLocation;

  /// The state of the floating cursor.
  final FloatingCursorDragState state;
}

/// The current text, selection, and composing state for editing a run of text.
@immutable
class TextEditingValue {
  /// Creates information for editing a run of text.
  ///
  /// The selection and composing range must be within the text. This is not
  /// checked during construction, and must be guaranteed by the caller.
  ///
  /// The default value of [selection] is `TextSelection.collapsed(offset: -1)`.
  /// This indicates that there is no selection at all.
  const TextEditingValue({
    this.text = '',
    this.selection = const TextSelection.collapsed(offset: -1),
    this.composing = TextRange.empty,
  });

  /// Creates an instance of this class from a JSON object.
  factory TextEditingValue.fromJSON(Map<String, dynamic> encoded) {
    final String text = encoded['text'] as String;
    final TextSelection selection = TextSelection(
      baseOffset: encoded['selectionBase'] as int? ?? -1,
      extentOffset: encoded['selectionExtent'] as int? ?? -1,
      affinity: _toTextAffinity(encoded['selectionAffinity'] as String?) ?? TextAffinity.downstream,
      isDirectional: encoded['selectionIsDirectional'] as bool? ?? false,
    );
    final TextRange composing = TextRange(
      start: encoded['composingBase'] as int? ?? -1,
      end: encoded['composingExtent'] as int? ?? -1,
    );
    assert(_textRangeIsValid(selection, text));
    assert(_textRangeIsValid(composing, text));
    return TextEditingValue(text: text, selection: selection, composing: composing);
  }

  /// The current text being edited.
  final String text;

  /// The range of text that is currently selected.
  ///
  /// When [selection] is a [TextSelection] that has the same non-negative
  /// `baseOffset` and `extentOffset`, the [selection] property represents the
  /// caret position.
  ///
  /// If the current [selection] has a negative `baseOffset` or `extentOffset`,
  /// then the text currently does not have a selection or a caret location, and
  /// most text editing operations that rely on the current selection (for
  /// instance, insert a character at the caret location) will do nothing.
  final TextSelection selection;

  /// The range of text that is still being composed.
  ///
  /// Composing regions are created by input methods (IMEs) to indicate the text
  /// within a certain range is provisional. For instance, the Android Gboard
  /// app's English keyboard puts the current word under the caret into a
  /// composing region to indicate the word is subject to autocorrect or
  /// prediction changes.
  ///
  /// Composing regions can also be used for performing multistage input, which
  /// is typically used by IMEs designed for phonetic keyboard to enter
  /// ideographic symbols. As an example, many CJK keyboards require the user to
  /// enter a Latin alphabet sequence and then convert it to CJK characters. On
  /// iOS, the default software keyboards do not have a dedicated view to show
  /// the unfinished Latin sequence, so it's displayed directly in the text
  /// field, inside of a composing region.
  ///
  /// The composing region should typically only be changed by the IME, or the
  /// user via interacting with the IME.
  ///
  /// If the range represented by this property is [TextRange.empty], then the
  /// text is not currently being composed.
  final TextRange composing;

  /// A value that corresponds to the empty string with no selection and no composing range.
  static const TextEditingValue empty = TextEditingValue();

  /// Creates a copy of this value but with the given fields replaced with the new values.
  TextEditingValue copyWith({String? text, TextSelection? selection, TextRange? composing}) {
    return TextEditingValue(
      text: text ?? this.text,
      selection: selection ?? this.selection,
      composing: composing ?? this.composing,
    );
  }

  /// Whether the [composing] range is a valid range within [text].
  ///
  /// Returns true if and only if the [composing] range is normalized, its start
  /// is greater than or equal to 0, and its end is less than or equal to
  /// [text]'s length.
  ///
  /// If this property is false while the [composing] range's `isValid` is true,
  /// it usually indicates the current [composing] range is invalid because of a
  /// programming error.
  bool get isComposingRangeValid =>
      composing.isValid && composing.isNormalized && composing.end <= text.length;

  /// Returns a new [TextEditingValue], which is this [TextEditingValue] with
  /// its [text] partially replaced by the `replacementString`.
  ///
  /// The `replacementRange` parameter specifies the range of the
  /// [TextEditingValue.text] that needs to be replaced.
  ///
  /// The `replacementString` parameter specifies the string to replace the
  /// given range of text with.
  ///
  /// This method also adjusts the selection range and the composing range of the
  /// resulting [TextEditingValue], such that they point to the same substrings
  /// as the corresponding ranges in the original [TextEditingValue]. For
  /// example, if the original [TextEditingValue] is "Hello world" with the word
  /// "world" selected, replacing "Hello" with a different string using this
  /// method will not change the selected word.
  ///
  /// This method does nothing if the given `replacementRange` is not
  /// [TextRange.isValid].
  TextEditingValue replaced(TextRange replacementRange, String replacementString) {
    if (!replacementRange.isValid) {
      return this;
    }
    final String newText = text.replaceRange(
      replacementRange.start,
      replacementRange.end,
      replacementString,
    );

    if (replacementRange.end - replacementRange.start == replacementString.length) {
      return copyWith(text: newText);
    }

    int adjustIndex(int originalIndex) {
      // The length added by adding the replacementString.
      final int replacedLength =
          originalIndex <= replacementRange.start && originalIndex < replacementRange.end
              ? 0
              : replacementString.length;
      // The length removed by removing the replacementRange.
      final int removedLength =
          originalIndex.clamp(replacementRange.start, replacementRange.end) -
          replacementRange.start;
      return originalIndex + replacedLength - removedLength;
    }

    final TextSelection adjustedSelection = TextSelection(
      baseOffset: adjustIndex(selection.baseOffset),
      extentOffset: adjustIndex(selection.extentOffset),
    );
    final TextRange adjustedComposing = TextRange(
      start: adjustIndex(composing.start),
      end: adjustIndex(composing.end),
    );
    assert(_textRangeIsValid(adjustedSelection, newText));
    assert(_textRangeIsValid(adjustedComposing, newText));
    return TextEditingValue(
      text: newText,
      selection: adjustedSelection,
      composing: adjustedComposing,
    );
  }

  /// Returns a representation of this object as a JSON object.
  Map<String, dynamic> toJSON() {
    assert(_textRangeIsValid(selection, text));
    assert(_textRangeIsValid(composing, text));
    return <String, dynamic>{
      'text': text,
      'selectionBase': selection.baseOffset,
      'selectionExtent': selection.extentOffset,
      'selectionAffinity': selection.affinity.toString(),
      'selectionIsDirectional': selection.isDirectional,
      'composingBase': composing.start,
      'composingExtent': composing.end,
    };
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, 'TextEditingValue')}(text: \u2524$text\u251C, selection: $selection, composing: $composing)';

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is TextEditingValue &&
        other.text == text &&
        other.selection == selection &&
        other.composing == composing;
  }

  @override
  int get hashCode => Object.hash(text.hashCode, selection.hashCode, composing.hashCode);

  // Verify that the given range is within the text.
  //
  // The verification can't be perform during the constructor of
  // [TextEditingValue], which are `const` and are allowed to retrieve
  // properties of [TextRange]s. [TextEditingValue] should perform this
  // wherever it is building other values (such as toJson) or is built in a
  // non-const way (such as fromJson).
  static bool _textRangeIsValid(TextRange range, String text) {
    if (range.start == -1 && range.end == -1) {
      return true;
    }
    assert(
      range.start >= 0 && range.start <= text.length,
      'Range start ${range.start} is out of text of length ${text.length}',
    );
    assert(
      range.end >= 0 && range.end <= text.length,
      'Range end ${range.end} is out of text of length ${text.length}',
    );
    return true;
  }
}

/// Indicates what triggered the change in selected text (including changes to
/// the cursor location).
enum SelectionChangedCause {
  /// The user tapped on the text and that caused the selection (or the location
  /// of the cursor) to change.
  tap,

  /// The user tapped twice in quick succession on the text and that caused
  /// the selection (or the location of the cursor) to change.
  doubleTap,

  /// The user long-pressed the text and that caused the selection (or the
  /// location of the cursor) to change.
  longPress,

  /// The user force-pressed the text and that caused the selection (or the
  /// location of the cursor) to change.
  forcePress,

  /// The user used the keyboard to change the selection or the location of the
  /// cursor.
  ///
  /// Keyboard-triggered selection changes may be caused by the IME as well as
  /// by accessibility tools (e.g. TalkBack on Android).
  keyboard,

  /// The user used the selection toolbar to change the selection or the
  /// location of the cursor.
  ///
  /// An example is when the user taps on select all in the tool bar.
  toolbar,

  /// The user used the mouse to change the selection by dragging over a piece
  /// of text.
  drag,

  /// The user used stylus handwriting to change the selection.
  ///
  /// Currently, this is only supported on iPadOS 14+ via the Scribble feature,
  /// or on Android API 34+ via the Scribe feature.
  stylusHandwriting;

  /// The user used stylus handwriting to change the selection.
  ///
  /// Currently, this is only supported on iPadOS 14+ via the Scribble feature,
  /// or on Android API 34+ via the Scribe feature.
  @Deprecated(
    'Use stylusHandwriting instead. '
    'This feature was deprecated after v3.28.0-0.1.pre.',
  )
  static const SelectionChangedCause scribble = stylusHandwriting;
}

/// A mixin for manipulating the selection, provided for toolbar or shortcut
/// keys.
mixin TextSelectionDelegate {
  /// Gets the current text input.
  TextEditingValue get textEditingValue;

  /// Indicates that the user has requested the delegate to replace its current
  /// text editing state with [value].
  ///
  /// The new [value] is treated as user input and thus may subject to input
  /// formatting.
  ///
  /// See also:
  ///
  /// * [EditableTextState.userUpdateTextEditingValue]: an implementation that
  ///   applies additional pre-processing to the specified [value], before
  ///   updating the text editing state.
  void userUpdateTextEditingValue(TextEditingValue value, SelectionChangedCause cause);

  /// Hides the text selection toolbar.
  ///
  /// By default, hideHandles is true, and the toolbar is hidden along with its
  /// handles. If hideHandles is set to false, then the toolbar will be hidden
  /// but the handles will remain.
  void hideToolbar([bool hideHandles = true]);

  /// Brings the provided [TextPosition] into the visible area of the text
  /// input.
  void bringIntoView(TextPosition position);

  /// Whether cut is enabled.
  bool get cutEnabled => true;

  /// Whether copy is enabled.
  bool get copyEnabled => true;

  /// Whether paste is enabled.
  bool get pasteEnabled => true;

  /// Whether select all is enabled.
  bool get selectAllEnabled => true;

  /// Whether look up is enabled.
  bool get lookUpEnabled => true;

  /// Whether search web is enabled.
  bool get searchWebEnabled => true;

  /// Whether share is enabled.
  bool get shareEnabled => true;

  /// Whether Live Text input is enabled.
  ///
  /// See also:
  ///  * [LiveText], where the availability of Live Text input can be obtained.
  ///  * [LiveTextInputStatusNotifier], where the status of Live Text can be listened to.
  bool get liveTextInputEnabled => false;

  /// Cut current selection to [Clipboard].
  ///
  /// If and only if [cause] is [SelectionChangedCause.toolbar], the toolbar
  /// will be hidden and the current selection will be scrolled into view.
  void cutSelection(SelectionChangedCause cause);

  /// Paste text from [Clipboard].
  ///
  /// If there is currently a selection, it will be replaced.
  ///
  /// If and only if [cause] is [SelectionChangedCause.toolbar], the toolbar
  /// will be hidden and the current selection will be scrolled into view.
  Future<void> pasteText(SelectionChangedCause cause);

  /// Set the current selection to contain the entire text value.
  ///
  /// If and only if [cause] is [SelectionChangedCause.toolbar], the selection
  /// will be scrolled into view.
  void selectAll(SelectionChangedCause cause);

  /// Copy current selection to [Clipboard].
  ///
  /// If [cause] is [SelectionChangedCause.toolbar], the position of
  /// [bringIntoView] to selection will be called and hide toolbar.
  void copySelection(SelectionChangedCause cause);
}

/// An interface to receive information from [TextInput].
///
/// If [TextInputConfiguration.enableDeltaModel] is set to true,
/// [DeltaTextInputClient] must be implemented instead of this class.
///
/// See also:
///
///  * [TextInput.attach]
///  * [EditableText], a [TextInputClient] implementation.
///  * [DeltaTextInputClient], a [TextInputClient] extension that receives
///    granular information from the platform's text input.
mixin TextInputClient {
  /// The current state of the [TextEditingValue] held by this client.
  TextEditingValue? get currentTextEditingValue;

  /// The [AutofillScope] this [TextInputClient] belongs to, if any.
  ///
  /// It should return null if this [TextInputClient] does not need autofill
  /// support. For a [TextInputClient] that supports autofill, returning null
  /// causes it to participate in autofill alone.
  ///
  /// See also:
  ///
  /// * [AutofillGroup], a widget that creates an [AutofillScope] for its
  ///   descendent autofillable [TextInputClient]s.
  AutofillScope? get currentAutofillScope;

  /// Requests that this client update its editing state to the given value.
  ///
  /// The new [value] is treated as user input and thus may subject to input
  /// formatting.
  void updateEditingValue(TextEditingValue value);

  /// Requests that this client perform the given action.
  void performAction(TextInputAction action);

  /// Notify client about new content insertion from Android keyboard.
  void insertContent(KeyboardInsertedContent content) {}

  /// Request from the input method that this client perform the given private
  /// command.
  ///
  /// This can be used to provide domain-specific features that are only known
  /// between certain input methods and their clients.
  ///
  /// See also:
  ///   * [performPrivateCommand](https://developer.android.com/reference/android/view/inputmethod/InputConnection#performPrivateCommand\(java.lang.String,%20android.os.Bundle\)),
  ///     which is the Android documentation for performPrivateCommand, used to
  ///     send a command from the input method.
  ///   * [sendAppPrivateCommand](https://developer.android.com/reference/android/view/inputmethod/InputMethodManager#sendAppPrivateCommand),
  ///     which is the Android documentation for sendAppPrivateCommand, used to
  ///     send a command to the input method.
  void performPrivateCommand(String action, Map<String, dynamic> data);

  /// Updates the floating cursor position and state.
  ///
  /// See also:
  ///
  ///  * [FloatingCursorDragState], which explains the floating cursor feature
  ///    in detail.
  void updateFloatingCursor(RawFloatingCursorPoint point);

  /// Requests that this client display a prompt rectangle for the given text range,
  /// to indicate the range of text that will be changed by a pending autocorrection.
  ///
  /// This method will only be called on iOS.
  void showAutocorrectionPromptRect(int start, int end);

  /// Platform notified framework of closed connection.
  ///
  /// [TextInputClient] should cleanup its connection and finalize editing.
  void connectionClosed();

  /// The framework calls this method to notify that the text input control has
  /// been changed.
  ///
  /// The [TextInputClient] may switch to the new text input control by hiding
  /// the old and showing the new input control.
  ///
  /// See also:
  ///
  ///  * [TextInputControl.hide], a method to hide the old input control.
  ///  * [TextInputControl.show], a method to show the new input control.
  void didChangeInputControl(TextInputControl? oldControl, TextInputControl? newControl) {}

  /// Requests that the client show the editing toolbar, for example when the
  /// platform changes the selection through a non-flutter method such as
  /// scribble.
  void showToolbar() {}

  /// Requests that the client add a text placeholder to reserve visual space
  /// in the text.
  ///
  /// For example, this is called when responding to UIKit requesting
  /// a text placeholder be added at the current selection, such as when
  /// requesting additional writing space with iPadOS14 Scribble.
  void insertTextPlaceholder(Size size) {}

  /// Requests that the client remove the text placeholder.
  void removeTextPlaceholder() {}

  /// Performs the specified MacOS-specific selector from the
  /// `NSStandardKeyBindingResponding` protocol or user-specified selector
  /// from `DefaultKeyBinding.Dict`.
  void performSelector(String selectorName) {}
}

/// An interface into iOS's stylus handwriting text input.
///
/// See also:
///
///  * [Scribe], which provides similar functionality for Android.
///  * [UIIndirectScribbleInteraction](https://developer.apple.com/documentation/uikit/uiindirectscribbleinteraction),
///    which is iOS's API for Scribble.
abstract class ScribbleClient {
  /// A unique identifier for this element.
  String get elementIdentifier;

  /// Called by the engine when the [ScribbleClient] should receive focus.
  ///
  /// For example, this method is called during a UIIndirectScribbleInteraction.
  void onScribbleFocus(Offset offset);

  /// Tests whether the [ScribbleClient] overlaps the given rectangle bounds.
  bool isInScribbleRect(Rect rect);

  /// The current bounds of the [ScribbleClient].
  Rect get bounds;
}

/// Represents a selection rect for a character and it's position in the text.
///
/// This is used to report the current text selection rect and position data
/// to the engine for Scribble support on iPadOS 14.
@immutable
class SelectionRect {
  /// Constructor for creating a [SelectionRect] from a text [position] and
  /// [bounds].
  const SelectionRect({
    required this.position,
    required this.bounds,
    this.direction = TextDirection.ltr,
  });

  /// The position of this selection rect within the text String.
  final int position;

  /// The rectangle representing the bounds of this selection rect within the
  /// currently focused [RenderEditable]'s coordinate space.
  final Rect bounds;

  /// The direction text flows within this selection rect.
  final TextDirection direction;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (runtimeType != other.runtimeType) {
      return false;
    }
    return other is SelectionRect &&
        other.position == position &&
        other.bounds == bounds &&
        other.direction == direction;
  }

  @override
  int get hashCode => Object.hash(position, bounds);

  @override
  String toString() => 'SelectionRect($position, $bounds)';
}

/// An interface to receive granular information from [TextInput].
///
/// See also:
///
///  * [TextInput.attach]
///  * [TextInputConfiguration], to opt-in to receive [TextEditingDelta]'s from
///    the platforms [TextInput] you must set [TextInputConfiguration.enableDeltaModel]
///    to true.
mixin DeltaTextInputClient implements TextInputClient {
  /// Requests that this client update its editing state by applying the deltas
  /// received from the engine.
  ///
  /// The list of [TextEditingDelta]'s are treated as changes that will be applied
  /// to the client's editing state. A change is any mutation to the raw text
  /// value, or any updates to the selection and/or composing region.
  ///
  /// {@tool snippet}
  /// This example shows what an implementation of this method could look like.
  ///
  /// ```dart
  /// class MyClient with DeltaTextInputClient {
  ///   TextEditingValue? _localValue;
  ///
  ///   @override
  ///   void updateEditingValueWithDeltas(List<TextEditingDelta> textEditingDeltas) {
  ///     if (_localValue == null) {
  ///       return;
  ///     }
  ///     TextEditingValue newValue = _localValue!;
  ///     for (final TextEditingDelta delta in textEditingDeltas) {
  ///       newValue = delta.apply(newValue);
  ///     }
  ///     _localValue = newValue;
  ///   }
  ///
  ///   // ...
  /// }
  /// ```
  /// {@end-tool}
  void updateEditingValueWithDeltas(List<TextEditingDelta> textEditingDeltas);
}

/// An interface for interacting with a text input control.
///
/// See also:
///
///  * [TextInput.attach], a method used to establish a [TextInputConnection]
///    between the system's text input and a [TextInputClient].
///  * [EditableText], a [TextInputClient] that connects to and interacts with
///    the system's text input using a [TextInputConnection].
class TextInputConnection {
  TextInputConnection._(this._client) : _id = _nextId++;

  Size? _cachedSize;
  Matrix4? _cachedTransform;
  Rect? _cachedRect;
  Rect? _cachedCaretRect;
  List<SelectionRect> _cachedSelectionRects = <SelectionRect>[];

  static int _nextId = 1;
  final int _id;

  /// Resets the internal ID counter for testing purposes.
  ///
  /// This call has no effect when asserts are disabled. Calling it from
  /// application code will likely break text input for the application.
  @visibleForTesting
  static void debugResetId({int to = 1}) {
    assert(() {
      _nextId = to;
      return true;
    }());
  }

  final TextInputClient _client;

  /// Whether this connection is currently interacting with the text input control.
  bool get attached => TextInput._instance._currentConnection == this;

  /// Whether there is currently a Scribble interaction in progress.
  ///
  /// This is used to make sure selection handles are shown when UIKit changes
  /// the selection during a Scribble interaction.
  bool get scribbleInProgress => TextInput._instance.scribbleInProgress;

  /// Requests that the text input control become visible.
  void show() {
    assert(attached);
    TextInput._instance._show();
  }

  /// Requests the system autofill UI to appear.
  ///
  /// Currently only works on Android. Other platforms do not respond to this
  /// message.
  ///
  /// See also:
  ///
  ///  * [EditableText], a [TextInputClient] that calls this method when focused.
  void requestAutofill() {
    assert(attached);
    TextInput._instance._requestAutofill();
  }

  /// Requests that the text input control update itself according to the new
  /// [TextInputConfiguration].
  void updateConfig(TextInputConfiguration configuration) {
    assert(attached);
    TextInput._instance._updateConfig(configuration);
  }

  /// Requests that the text input control change its internal state to match
  /// the given state.
  void setEditingState(TextEditingValue value) {
    assert(attached);
    TextInput._instance._setEditingState(value);
  }

  /// Send the size and transform of the editable text to engine.
  ///
  /// The values are sent as platform messages so they can be used on web for
  /// example to correctly position and size the html input field.
  ///
  /// 1. [editableBoxSize]: size of the render editable box.
  ///
  /// 2. [transform]: a matrix that maps the local paint coordinate system
  ///                 to the [PipelineOwner.rootNode].
  void setEditableSizeAndTransform(Size editableBoxSize, Matrix4 transform) {
    if (editableBoxSize != _cachedSize || transform != _cachedTransform) {
      _cachedSize = editableBoxSize;
      _cachedTransform = transform;
      TextInput._instance._setEditableSizeAndTransform(editableBoxSize, transform);
    }
  }

  /// Send the smallest rect that covers the text in the client that's currently
  /// being composed.
  ///
  /// If any of the 4 coordinates of the given [Rect] is not finite, a [Rect] of
  /// size (-1, -1) will be sent instead.
  ///
  /// This information is used for positioning the IME candidates menu on each
  /// platform.
  void setComposingRect(Rect rect) {
    if (rect == _cachedRect) {
      return;
    }
    _cachedRect = rect;
    final Rect validRect = rect.isFinite ? rect : Offset.zero & const Size(-1, -1);
    TextInput._instance._setComposingTextRect(validRect);
  }

  /// Sends the coordinates of caret rect. This is used on macOS for positioning
  /// the accent selection menu.
  void setCaretRect(Rect rect) {
    if (rect == _cachedCaretRect) {
      return;
    }
    _cachedCaretRect = rect;
    final Rect validRect = rect.isFinite ? rect : Offset.zero & const Size(-1, -1);
    TextInput._instance._setCaretRect(validRect);
  }

  /// Send the bounding boxes of the current selected glyphs in the client to
  /// the platform's text input plugin.
  ///
  /// These are used by the engine during a UIDirectScribbleInteraction.
  void setSelectionRects(List<SelectionRect> selectionRects) {
    if (!listEquals(_cachedSelectionRects, selectionRects)) {
      _cachedSelectionRects = selectionRects;
      TextInput._instance._setSelectionRects(selectionRects);
    }
  }

  /// Send text styling information.
  ///
  /// This information is used by the Flutter Web Engine to change the style
  /// of the hidden native input's content. Hence, the content size will match
  /// to the size of the editable widget's content.
  void setStyle({
    required String? fontFamily,
    required double? fontSize,
    required FontWeight? fontWeight,
    required TextDirection textDirection,
    required TextAlign textAlign,
  }) {
    assert(attached);

    TextInput._instance._setStyle(
      fontFamily: fontFamily,
      fontSize: fontSize,
      fontWeight: fontWeight,
      textDirection: textDirection,
      textAlign: textAlign,
    );
  }

  /// Stop interacting with the text input control.
  ///
  /// After calling this method, the text input control might disappear if no
  /// other client attaches to it within this animation frame.
  void close() {
    if (attached) {
      TextInput._instance._clearClient();
    }
    assert(!attached);
  }

  /// Platform sent a notification informing the connection is closed.
  ///
  /// [TextInputConnection] should clean current client connection.
  void connectionClosedReceived() {
    TextInput._instance._currentConnection = null;
    assert(!attached);
  }
}

TextInputAction _toTextInputAction(String action) {
  return switch (action) {
    'TextInputAction.none' => TextInputAction.none,
    'TextInputAction.unspecified' => TextInputAction.unspecified,
    'TextInputAction.go' => TextInputAction.go,
    'TextInputAction.search' => TextInputAction.search,
    'TextInputAction.send' => TextInputAction.send,
    'TextInputAction.next' => TextInputAction.next,
    'TextInputAction.previous' => TextInputAction.previous,
    'TextInputAction.continueAction' => TextInputAction.continueAction,
    'TextInputAction.join' => TextInputAction.join,
    'TextInputAction.route' => TextInputAction.route,
    'TextInputAction.emergencyCall' => TextInputAction.emergencyCall,
    'TextInputAction.done' => TextInputAction.done,
    'TextInputAction.newline' => TextInputAction.newline,
    _ =>
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('Unknown text input action: $action'),
      ]),
  };
}

FloatingCursorDragState _toTextCursorAction(String state) {
  return switch (state) {
    'FloatingCursorDragState.start' => FloatingCursorDragState.Start,
    'FloatingCursorDragState.update' => FloatingCursorDragState.Update,
    'FloatingCursorDragState.end' => FloatingCursorDragState.End,
    _ =>
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('Unknown text cursor action: $state'),
      ]),
  };
}

RawFloatingCursorPoint _toTextPoint(FloatingCursorDragState state, Map<String, dynamic> encoded) {
  assert(
    encoded['X'] != null,
    'You must provide a value for the horizontal location of the floating cursor.',
  );
  assert(
    encoded['Y'] != null,
    'You must provide a value for the vertical location of the floating cursor.',
  );
  final Offset offset =
      state == FloatingCursorDragState.Update
          ? Offset((encoded['X'] as num).toDouble(), (encoded['Y'] as num).toDouble())
          : Offset.zero;
  return RawFloatingCursorPoint(offset: offset, state: state);
}

/// An low-level interface to the system's text input control.
///
/// To start interacting with the system's text input control, call [attach] to
/// establish a [TextInputConnection] between the system's text input control
/// and a [TextInputClient]. The majority of commands available for
/// interacting with the text input control reside in the returned
/// [TextInputConnection]. The communication between the system text input and
/// the [TextInputClient] is asynchronous.
///
/// The platform text input plugin (which represents the system's text input)
/// and the [TextInputClient] usually maintain their own text editing states
/// ([TextEditingValue]) separately. They must be kept in sync as long as the
/// [TextInputClient] is connected. The following methods can be used to send
/// [TextEditingValue] to update the other party, when either party's text
/// editing states change:
///
/// * The [TextInput.attach] method allows a [TextInputClient] to establish a
///   connection to the text input. An optional field in its `configuration`
///   parameter can be used to specify an initial value for the platform text
///   input plugin's [TextEditingValue].
///
/// * The [TextInputClient] sends its [TextEditingValue] to the platform text
///   input plugin using [TextInputConnection.setEditingState].
///
/// * The platform text input plugin sends its [TextEditingValue] to the
///   connected [TextInputClient] via a "TextInput.setEditingState" message.
///
/// * When autofill happens on a disconnected [TextInputClient], the platform
///   text input plugin sends the [TextEditingValue] to the connected
///   [TextInputClient]'s [AutofillScope], and the [AutofillScope] will further
///   relay the value to the correct [TextInputClient].
///
/// When synchronizing the [TextEditingValue]s, the communication may get stuck
/// in an infinite when both parties are trying to send their own update. To
/// mitigate the problem, only [TextInputClient]s are allowed to alter the
/// received [TextEditingValue]s while platform text input plugins are to accept
/// the received [TextEditingValue]s unmodified. More specifically:
///
/// * When a [TextInputClient] receives a new [TextEditingValue] from the
///   platform text input plugin, it's allowed to modify the value (for example,
///   apply [TextInputFormatter]s). If it decides to do so, it must send the
///   updated [TextEditingValue] back to the platform text input plugin to keep
///   the [TextEditingValue]s in sync.
///
/// * When the platform text input plugin receives a new value from the
///   connected [TextInputClient], it must accept the new value as-is, to avoid
///   sending back an updated value.
///
/// See also:
///
///  * [TextField], a widget in which the user may enter text.
///  * [EditableText], a [TextInputClient] that connects to [TextInput] when it
///    wants to take user input from the keyboard.
class TextInput {
  TextInput._() {
    _channel = SystemChannels.textInput;
    _channel.setMethodCallHandler(_loudlyHandleTextInputInvocation);
  }

  /// Set the [MethodChannel] used to communicate with the system's text input
  /// control.
  ///
  /// This is only meant for testing within the Flutter SDK. Changing this
  /// will break the ability to input text. This has no effect if asserts are
  /// disabled.
  @visibleForTesting
  static void setChannel(MethodChannel newChannel) {
    assert(() {
      _instance._channel =
          newChannel..setMethodCallHandler(_instance._loudlyHandleTextInputInvocation);
      return true;
    }());
  }

  static final TextInput _instance = TextInput._();

  static void _addInputControl(TextInputControl control) {
    if (control != _PlatformTextInputControl.instance) {
      _instance._inputControls.add(control);
    }
  }

  static void _removeInputControl(TextInputControl control) {
    if (control != _PlatformTextInputControl.instance) {
      _instance._inputControls.remove(control);
    }
  }

  /// Sets the current text input control.
  ///
  /// The current text input control receives text input state changes and visual
  /// text input control requests, such as showing and hiding the input control,
  /// from the framework.
  ///
  /// Setting the current text input control as `null` removes the visual text
  /// input control.
  ///
  /// See also:
  ///
  ///  * [TextInputControl], an interface for implementing text input controls.
  ///  * [TextInput.restorePlatformInputControl], a method to restore the default
  ///    platform text input control.
  static void setInputControl(TextInputControl? newControl) {
    final TextInputControl? oldControl = _instance._currentControl;
    if (newControl == oldControl) {
      return;
    }
    if (newControl != null) {
      _addInputControl(newControl);
    }
    if (oldControl != null) {
      _removeInputControl(oldControl);
    }
    _instance._currentControl = newControl;
    final TextInputClient? client = _instance._currentConnection?._client;
    client?.didChangeInputControl(oldControl, newControl);
  }

  /// Restores the default platform text input control.
  ///
  /// See also:
  ///
  /// * [TextInput.setInputControl], a method to set a custom input
  ///   control, or to remove the visual input control.
  static void restorePlatformInputControl() {
    setInputControl(_PlatformTextInputControl.instance);
  }

  TextInputControl? _currentControl = _PlatformTextInputControl.instance;
  final Set<TextInputControl> _inputControls = <TextInputControl>{
    _PlatformTextInputControl.instance,
  };

  static const List<TextInputAction> _androidSupportedInputActions = <TextInputAction>[
    TextInputAction.none,
    TextInputAction.unspecified,
    TextInputAction.done,
    TextInputAction.send,
    TextInputAction.go,
    TextInputAction.search,
    TextInputAction.next,
    TextInputAction.previous,
    TextInputAction.newline,
  ];

  static const List<TextInputAction> _iOSSupportedInputActions = <TextInputAction>[
    TextInputAction.unspecified,
    TextInputAction.done,
    TextInputAction.send,
    TextInputAction.go,
    TextInputAction.search,
    TextInputAction.next,
    TextInputAction.newline,
    TextInputAction.continueAction,
    TextInputAction.join,
    TextInputAction.route,
    TextInputAction.emergencyCall,
  ];

  /// Ensure that a [TextInput] instance has been set up so that the platform
  /// can handle messages on the text input method channel.
  static void ensureInitialized() {
    _instance; // ignore: unnecessary_statements
  }

  /// Begin interacting with the text input control.
  ///
  /// Calling this function helps multiple clients coordinate about which one is
  /// currently interacting with the text input control. The returned
  /// [TextInputConnection] provides an interface for actually interacting with
  /// the text input control.
  ///
  /// A client that no longer wishes to interact with the text input control
  /// should call [TextInputConnection.close] on the returned
  /// [TextInputConnection].
  static TextInputConnection attach(TextInputClient client, TextInputConfiguration configuration) {
    final TextInputConnection connection = TextInputConnection._(client);
    _instance._attach(connection, configuration);
    return connection;
  }

  // This method actually notifies the embedding of the client. It is utilized
  // by [attach] and by [_handleTextInputInvocation] for the
  // `TextInputClient.requestExistingInputState` method.
  void _attach(TextInputConnection connection, TextInputConfiguration configuration) {
    assert(_debugEnsureInputActionWorksOnPlatform(configuration.inputAction));
    _currentConnection = connection;
    _currentConfiguration = configuration;
    _setClient(connection._client, configuration);
  }

  static bool _debugEnsureInputActionWorksOnPlatform(TextInputAction inputAction) {
    assert(() {
      if (kIsWeb) {
        // TODO(flutterweb): what makes sense here?
        return true;
      }
      if (Platform.isIOS) {
        assert(
          _iOSSupportedInputActions.contains(inputAction),
          'The requested TextInputAction "$inputAction" is not supported on iOS.',
        );
      } else if (Platform.isAndroid) {
        assert(
          _androidSupportedInputActions.contains(inputAction),
          'The requested TextInputAction "$inputAction" is not supported on Android.',
        );
      }
      return true;
    }());
    return true;
  }

  late MethodChannel _channel;

  TextInputConnection? _currentConnection;
  late TextInputConfiguration _currentConfiguration;

  final Map<String, ScribbleClient> _scribbleClients = <String, ScribbleClient>{};
  bool _scribbleInProgress = false;

  /// Used for testing within the Flutter SDK to get the currently registered [ScribbleClient] list.
  @visibleForTesting
  static Map<String, ScribbleClient> get scribbleClients => TextInput._instance._scribbleClients;

  /// Returns true if a scribble interaction is currently happening.
  bool get scribbleInProgress => _scribbleInProgress;

  Future<dynamic> _loudlyHandleTextInputInvocation(MethodCall call) async {
    try {
      return await _handleTextInputInvocation(call);
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'services library',
          context: ErrorDescription('during method call ${call.method}'),
          informationCollector:
              () => <DiagnosticsNode>[
                DiagnosticsProperty<MethodCall>(
                  'call',
                  call,
                  style: DiagnosticsTreeStyle.errorProperty,
                ),
              ],
        ),
      );
      rethrow;
    }
  }

  Future<dynamic> _handleTextInputInvocation(MethodCall methodCall) async {
    final String method = methodCall.method;
    switch (method) {
      case 'TextInputClient.focusElement':
        final List<dynamic> args = methodCall.arguments as List<dynamic>;
        _scribbleClients[args[0]]?.onScribbleFocus(
          Offset((args[1] as num).toDouble(), (args[2] as num).toDouble()),
        );
        return;
      case 'TextInputClient.requestElementsInRect':
        final List<double> args =
            (methodCall.arguments as List<dynamic>)
                .cast<num>()
                .map<double>((num value) => value.toDouble())
                .toList();
        return _scribbleClients.keys
            .where((String elementIdentifier) {
              final Rect rect = Rect.fromLTWH(args[0], args[1], args[2], args[3]);
              if (!(_scribbleClients[elementIdentifier]?.isInScribbleRect(rect) ?? false)) {
                return false;
              }
              final Rect bounds = _scribbleClients[elementIdentifier]?.bounds ?? Rect.zero;
              return !(bounds == Rect.zero || bounds.hasNaN || bounds.isInfinite);
            })
            .map((String elementIdentifier) {
              final Rect bounds = _scribbleClients[elementIdentifier]!.bounds;
              return <dynamic>[
                elementIdentifier,
                ...<dynamic>[bounds.left, bounds.top, bounds.width, bounds.height],
              ];
            })
            .toList();
      case 'TextInputClient.scribbleInteractionBegan':
        _scribbleInProgress = true;
        return;
      case 'TextInputClient.scribbleInteractionFinished':
        _scribbleInProgress = false;
        return;
    }
    if (_currentConnection == null) {
      return;
    }

    // The requestExistingInputState request needs to be handled regardless of
    // the client ID, as long as we have a _currentConnection.
    if (method == 'TextInputClient.requestExistingInputState') {
      _attach(_currentConnection!, _currentConfiguration);
      final TextEditingValue? editingValue = _currentConnection!._client.currentTextEditingValue;
      if (editingValue != null) {
        _setEditingState(editingValue);
      }
      return;
    }

    final List<dynamic> args = methodCall.arguments as List<dynamic>;

    // The updateEditingStateWithTag request (autofill) can come up even to a
    // text field that doesn't have a connection.
    if (method == 'TextInputClient.updateEditingStateWithTag') {
      final TextInputClient client = _currentConnection!._client;
      final AutofillScope? scope = client.currentAutofillScope;
      final Map<String, dynamic> editingValue = args[1] as Map<String, dynamic>;
      for (final String tag in editingValue.keys) {
        final TextEditingValue textEditingValue = TextEditingValue.fromJSON(
          editingValue[tag] as Map<String, dynamic>,
        );
        final AutofillClient? client = scope?.getAutofillClient(tag);
        if (client != null && client.textInputConfiguration.autofillConfiguration.enabled) {
          client.autofill(textEditingValue);
        }
      }

      return;
    }

    final int client = args[0] as int;
    if (client != _currentConnection!._id) {
      // If the client IDs don't match, the incoming message was for a different
      // client.
      bool debugAllowAnyway = false;
      assert(() {
        // In debug builds we allow "-1" as a magical client ID that ignores
        // this verification step so that tests can always get through, even
        // when they are not mocking the engine side of text input.
        if (client == -1) {
          debugAllowAnyway = true;
        }
        return true;
      }());
      if (!debugAllowAnyway) {
        return;
      }
    }

    switch (method) {
      case 'TextInputClient.updateEditingState':
        final TextEditingValue value = TextEditingValue.fromJSON(args[1] as Map<String, dynamic>);
        TextInput._instance._updateEditingValue(value, exclude: _PlatformTextInputControl.instance);
      case 'TextInputClient.updateEditingStateWithDeltas':
        assert(
          _currentConnection!._client is DeltaTextInputClient,
          'You must be using a DeltaTextInputClient if TextInputConfiguration.enableDeltaModel is set to true',
        );
        final Map<String, dynamic> encoded = args[1] as Map<String, dynamic>;
        final List<TextEditingDelta> deltas = <TextEditingDelta>[
          for (final dynamic encodedDelta in encoded['deltas'] as List<dynamic>)
            TextEditingDelta.fromJSON(encodedDelta as Map<String, dynamic>),
        ];

        (_currentConnection!._client as DeltaTextInputClient).updateEditingValueWithDeltas(deltas);
      case 'TextInputClient.performAction':
        if (args[1] as String == 'TextInputAction.commitContent') {
          final KeyboardInsertedContent content = KeyboardInsertedContent.fromJson(
            args[2] as Map<String, dynamic>,
          );
          _currentConnection!._client.insertContent(content);
        } else {
          _currentConnection!._client.performAction(_toTextInputAction(args[1] as String));
        }
      case 'TextInputClient.performSelectors':
        final List<String> selectors = (args[1] as List<dynamic>).cast<String>();
        selectors.forEach(_currentConnection!._client.performSelector);
      case 'TextInputClient.performPrivateCommand':
        final Map<String, dynamic> firstArg = args[1] as Map<String, dynamic>;
        _currentConnection!._client.performPrivateCommand(
          firstArg['action'] as String,
          firstArg['data'] == null ? <String, dynamic>{} : firstArg['data'] as Map<String, dynamic>,
        );
      case 'TextInputClient.updateFloatingCursor':
        _currentConnection!._client.updateFloatingCursor(
          _toTextPoint(_toTextCursorAction(args[1] as String), args[2] as Map<String, dynamic>),
        );
      case 'TextInputClient.onConnectionClosed':
        _currentConnection!._client.connectionClosed();
      case 'TextInputClient.showAutocorrectionPromptRect':
        _currentConnection!._client.showAutocorrectionPromptRect(args[1] as int, args[2] as int);
      case 'TextInputClient.showToolbar':
        _currentConnection!._client.showToolbar();
      case 'TextInputClient.insertTextPlaceholder':
        _currentConnection!._client.insertTextPlaceholder(
          Size((args[1] as num).toDouble(), (args[2] as num).toDouble()),
        );
      case 'TextInputClient.removeTextPlaceholder':
        _currentConnection!._client.removeTextPlaceholder();
      default:
        throw MissingPluginException();
    }
  }

  bool _hidePending = false;

  void _scheduleHide() {
    if (_hidePending) {
      return;
    }
    _hidePending = true;

    // Schedule a deferred task that hides the text input. If someone else
    // shows the keyboard during this update cycle, then the task will do
    // nothing.
    scheduleMicrotask(() {
      _hidePending = false;
      if (_currentConnection == null) {
        _hide();
      }
    });
  }

  void _setClient(TextInputClient client, TextInputConfiguration configuration) {
    for (final TextInputControl control in _inputControls) {
      control.attach(client, configuration);
    }
  }

  void _clearClient() {
    final TextInputClient client = _currentConnection!._client;
    for (final TextInputControl control in _inputControls) {
      control.detach(client);
    }
    _currentConnection = null;
    _scheduleHide();
  }

  void _updateConfig(TextInputConfiguration configuration) {
    for (final TextInputControl control in _inputControls) {
      control.updateConfig(configuration);
    }
  }

  void _setEditingState(TextEditingValue value) {
    for (final TextInputControl control in _inputControls) {
      control.setEditingState(value);
    }
  }

  void _show() {
    for (final TextInputControl control in _inputControls) {
      control.show();
    }
  }

  void _hide() {
    for (final TextInputControl control in _inputControls) {
      control.hide();
    }
  }

  void _setEditableSizeAndTransform(Size editableBoxSize, Matrix4 transform) {
    for (final TextInputControl control in _inputControls) {
      control.setEditableSizeAndTransform(editableBoxSize, transform);
    }
  }

  void _setComposingTextRect(Rect rect) {
    for (final TextInputControl control in _inputControls) {
      control.setComposingRect(rect);
    }
  }

  void _setCaretRect(Rect rect) {
    for (final TextInputControl control in _inputControls) {
      control.setCaretRect(rect);
    }
  }

  void _setSelectionRects(List<SelectionRect> selectionRects) {
    for (final TextInputControl control in _inputControls) {
      control.setSelectionRects(selectionRects);
    }
  }

  void _setStyle({
    required String? fontFamily,
    required double? fontSize,
    required FontWeight? fontWeight,
    required TextDirection textDirection,
    required TextAlign textAlign,
  }) {
    for (final TextInputControl control in _inputControls) {
      control.setStyle(
        fontFamily: fontFamily,
        fontSize: fontSize,
        fontWeight: fontWeight,
        textDirection: textDirection,
        textAlign: textAlign,
      );
    }
  }

  void _requestAutofill() {
    for (final TextInputControl control in _inputControls) {
      control.requestAutofill();
    }
  }

  void _updateEditingValue(TextEditingValue value, {TextInputControl? exclude}) {
    if (_currentConnection == null) {
      return;
    }

    for (final TextInputControl control in _instance._inputControls) {
      if (control != exclude) {
        control.setEditingState(value);
      }
    }
    _instance._currentConnection!._client.updateEditingValue(value);
  }

  /// Updates the editing value of the attached input client.
  ///
  /// This method should be called by the text input control implementation to
  /// send editing value updates to the attached input client.
  static void updateEditingValue(TextEditingValue value) {
    _instance._updateEditingValue(value, exclude: _instance._currentControl);
  }

  /// Finishes the current autofill context, and potentially saves the user
  /// input for future use if `shouldSave` is true.
  ///
  /// Typically, this method should be called when the user has finalized their
  /// input. For example, in a [Form], it's typically done immediately before or
  /// after its content is submitted.
  ///
  /// The topmost [AutofillGroup]s also call [finishAutofillContext]
  /// automatically when they are disposed. The default behavior can be
  /// overridden in [AutofillGroup.onDisposeAction].
  ///
  /// {@template flutter.services.TextInput.finishAutofillContext}
  /// An autofill context is a collection of input fields that live in the
  /// platform's text input plugin. The platform is encouraged to save the user
  /// input stored in the current autofill context before the context is
  /// destroyed, when [TextInput.finishAutofillContext] is called with
  /// `shouldSave` set to true.
  ///
  /// Currently, there can only be at most one autofill context at any given
  /// time. When any input field in an [AutofillGroup] requests for autofill
  /// (which is done automatically when an autofillable [EditableText] gains
  /// focus), the current autofill context will merge the content of that
  /// [AutofillGroup] into itself. When there isn't an existing autofill context,
  /// one will be created to hold the newly added input fields from the group.
  ///
  /// Once added to an autofill context, an input field will stay in the context
  /// until the context is destroyed. To prevent leaks, call
  /// [TextInput.finishAutofillContext] to signal the text input plugin that the
  /// user has finalized their input in the current autofill context. The
  /// platform text input plugin either encourages or discourages the platform
  /// from saving the user input based on the value of the `shouldSave`
  /// parameter. The platform usually shows a "Save for autofill?" prompt for
  /// user confirmation.
  /// {@endtemplate}
  ///
  /// On many platforms, calling [finishAutofillContext] shows the save user
  /// input dialog and disrupts the user's flow. Ideally the dialog should only
  /// be shown no more than once for every screen. Consider removing premature
  /// [finishAutofillContext] calls to prevent showing the save user input UI
  /// too frequently. However, calling [finishAutofillContext] when there's no
  /// existing autofill context usually does not bring up the save user input
  /// UI.
  ///
  /// See also:
  ///
  /// * [EditableText.autofillHints] for autofill save troubleshooting tips.
  /// * [AutofillGroup.onDisposeAction], a configurable action that runs when a
  ///   topmost [AutofillGroup] is getting disposed.
  static void finishAutofillContext({bool shouldSave = true}) {
    for (final TextInputControl control in TextInput._instance._inputControls) {
      control.finishAutofillContext(shouldSave: shouldSave);
    }
  }

  /// Registers a [ScribbleClient] with [elementIdentifier] that can be focused
  /// by the engine.
  ///
  /// For example, the registered [ScribbleClient] list is used to respond to
  /// UIIndirectScribbleInteraction on an iPad.
  static void registerScribbleElement(String elementIdentifier, ScribbleClient scribbleClient) {
    TextInput._instance._scribbleClients[elementIdentifier] = scribbleClient;
  }

  /// Unregisters a [ScribbleClient] with [elementIdentifier].
  static void unregisterScribbleElement(String elementIdentifier) {
    TextInput._instance._scribbleClients.remove(elementIdentifier);
  }
}

/// An interface for implementing text input controls that receive text editing
/// state changes and visual input control requests.
///
/// Editing state changes and input control requests are sent by the framework
/// when the editing state of the attached text input client changes, or it
/// requests the input control to be shown or hidden, for example.
///
/// The input control can be installed with [TextInput.setInputControl], and the
/// default platform text input control can be restored with
/// [TextInput.restorePlatformInputControl].
///
/// The [TextInputControl] class must be extended. [TextInputControl]
/// implementations should call [TextInput.updateEditingValue] to send user
/// input to the attached input client.
///
/// {@tool dartpad}
/// This example illustrates a basic [TextInputControl] implementation.
///
/// ** See code in examples/api/lib/services/text_input/text_input_control.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [TextInput.setInputControl], a method to install a custom text input control.
///  * [TextInput.restorePlatformInputControl], a method to restore the default
///    platform text input control.
///  * [TextInput.updateEditingValue], a method to send user input to
///    the framework.
mixin TextInputControl {
  /// Requests the text input control to attach to the given input client.
  ///
  /// This method is called when a text input client is attached. The input
  /// control should update its configuration to match the client's configuration.
  void attach(TextInputClient client, TextInputConfiguration configuration) {}

  /// Requests the text input control to detach from the given input client.
  ///
  /// This method is called when a text input client is detached. The input
  /// control should release any resources allocated for the client.
  void detach(TextInputClient client) {}

  /// Requests that the text input control is shown.
  ///
  /// This method is called when the input control should become visible.
  void show() {}

  /// Requests that the text input control is hidden.
  ///
  /// This method is called when the input control should hide.
  void hide() {}

  /// Informs the text input control about input configuration changes.
  ///
  /// This method is called when the configuration of the attached input client
  /// has changed.
  void updateConfig(TextInputConfiguration configuration) {}

  /// Informs the text input control about editing state changes.
  ///
  /// This method is called when the editing state of the attached input client
  /// has changed.
  void setEditingState(TextEditingValue value) {}

  /// Informs the text input control about client position changes.
  ///
  /// This method is called on when the input control should position itself in
  /// relation to the attached input client.
  void setEditableSizeAndTransform(Size editableBoxSize, Matrix4 transform) {}

  /// Informs the text input control about composing area changes.
  ///
  /// This method is called when the attached input client's composing area
  /// changes.
  void setComposingRect(Rect rect) {}

  /// Informs the text input control about caret area changes.
  ///
  /// This method is called when the attached input client's caret area
  /// changes.
  void setCaretRect(Rect rect) {}

  /// Informs the text input control about selection area changes.
  ///
  /// This method is called when the attached input client's selection area
  /// changes.
  void setSelectionRects(List<SelectionRect> selectionRects) {}

  /// Informs the text input control about text style changes.
  ///
  /// This method is called on the when the attached input client's text style
  /// changes.
  void setStyle({
    required String? fontFamily,
    required double? fontSize,
    required FontWeight? fontWeight,
    required TextDirection textDirection,
    required TextAlign textAlign,
  }) {}

  /// Requests autofill from the text input control.
  ///
  /// This method is called when the autofill UI should appear.
  void requestAutofill() {}

  /// Requests that the autofill context is finalized.
  ///
  /// See also:
  ///
  ///  * [TextInput.finishAutofillContext]
  void finishAutofillContext({bool shouldSave = true}) {}
}

/// Provides access to the platform text input control.
class _PlatformTextInputControl with TextInputControl {
  _PlatformTextInputControl._();

  /// The shared instance of [_PlatformTextInputControl].
  static final _PlatformTextInputControl instance = _PlatformTextInputControl._();

  MethodChannel get _channel => TextInput._instance._channel;

  Map<String, dynamic> _configurationToJson(TextInputConfiguration configuration) {
    final Map<String, dynamic> json = configuration.toJson();
    if (TextInput._instance._currentControl != _PlatformTextInputControl.instance) {
      final Map<String, dynamic> none = TextInputType.none.toJson();
      // See: https://github.com/flutter/flutter/issues/125875
      // On Web engine, use isMultiline to create <input> or <textarea> element
      // When there's a custom [TextInputControl] installed.
      // It's only needed When there's a custom [TextInputControl] installed.
      if (kIsWeb) {
        none['isMultiline'] = configuration.inputType == TextInputType.multiline;
      }
      json['inputType'] = none;
    }
    return json;
  }

  @override
  void attach(TextInputClient client, TextInputConfiguration configuration) {
    _channel.invokeMethod<void>('TextInput.setClient', <Object>[
      TextInput._instance._currentConnection!._id,
      _configurationToJson(configuration),
    ]);
  }

  @override
  void detach(TextInputClient client) {
    _channel.invokeMethod<void>('TextInput.clearClient');
  }

  @override
  void updateConfig(TextInputConfiguration configuration) {
    _channel.invokeMethod<void>('TextInput.updateConfig', _configurationToJson(configuration));
  }

  @override
  void setEditingState(TextEditingValue value) {
    _channel.invokeMethod<void>('TextInput.setEditingState', value.toJSON());
  }

  @override
  void show() {
    _channel.invokeMethod<void>('TextInput.show');
  }

  @override
  void hide() {
    _channel.invokeMethod<void>('TextInput.hide');
  }

  @override
  void setEditableSizeAndTransform(Size editableBoxSize, Matrix4 transform) {
    _channel.invokeMethod<void>('TextInput.setEditableSizeAndTransform', <String, dynamic>{
      'width': editableBoxSize.width,
      'height': editableBoxSize.height,
      'transform': transform.storage,
    });
  }

  @override
  void setComposingRect(Rect rect) {
    _channel.invokeMethod<void>('TextInput.setMarkedTextRect', <String, dynamic>{
      'width': rect.width,
      'height': rect.height,
      'x': rect.left,
      'y': rect.top,
    });
  }

  @override
  void setCaretRect(Rect rect) {
    _channel.invokeMethod<void>('TextInput.setCaretRect', <String, dynamic>{
      'width': rect.width,
      'height': rect.height,
      'x': rect.left,
      'y': rect.top,
    });
  }

  @override
  void setSelectionRects(List<SelectionRect> selectionRects) {
    _channel.invokeMethod<void>(
      'TextInput.setSelectionRects',
      selectionRects.map((SelectionRect rect) {
        return <num>[
          rect.bounds.left,
          rect.bounds.top,
          rect.bounds.width,
          rect.bounds.height,
          rect.position,
          rect.direction.index,
        ];
      }).toList(),
    );
  }

  @override
  void setStyle({
    required String? fontFamily,
    required double? fontSize,
    required FontWeight? fontWeight,
    required TextDirection textDirection,
    required TextAlign textAlign,
  }) {
    _channel.invokeMethod<void>('TextInput.setStyle', <String, dynamic>{
      'fontFamily': fontFamily,
      'fontSize': fontSize,
      'fontWeightIndex': fontWeight?.index,
      'textAlignIndex': textAlign.index,
      'textDirectionIndex': textDirection.index,
    });
  }

  @override
  void requestAutofill() {
    _channel.invokeMethod<void>('TextInput.requestAutofill');
  }

  @override
  void finishAutofillContext({bool shouldSave = true}) {
    _channel.invokeMethod<void>('TextInput.finishAutofillContext', shouldSave);
  }
}

/// Allows access to the system context menu.
///
/// The context menu is the menu that appears, for example, when doing text
/// selection. Flutter typically draws this menu itself, but this class deals
/// with the platform-rendered context menu.
///
/// Only one instance can be visible at a time. Calling [show] while the system
/// context menu is already visible will hide it and show it again at the new
/// [Rect]. An instance that is hidden is informed via [onSystemHide].
///
/// Currently this system context menu is bound to text input. The buttons that
/// are shown and the actions they perform are dependent on the currently
/// active [TextInputConnection]. Using this without an active
/// [TextInputConnection] is a noop.
///
/// Call [dispose] when no longer needed.
///
/// See also:
///
///  * [ContextMenuController], which controls Flutter-drawn context menus.
///  * [SystemContextMenu], which wraps this functionality in a widget.
///  * [MediaQuery.maybeSupportsShowingSystemContextMenu], which indicates
///    whether the system context menu is supported.
class SystemContextMenuController with SystemContextMenuClient {
  /// Creates an instance of [SystemContextMenuController].
  ///
  /// Not shown until [show] is called.
  SystemContextMenuController({this.onSystemHide}) {
    ServicesBinding.systemContextMenuClient = this;
  }

  /// Called when the system has hidden the context menu.
  ///
  /// For example, tapping outside of the context menu typically causes the
  /// system to hide it directly. Flutter is made aware that the context menu is
  /// no longer visible through this callback.
  ///
  /// This is not called when [show]ing a new system context menu causes another
  /// to be hidden.
  final VoidCallback? onSystemHide;

  static const MethodChannel _channel = SystemChannels.platform;

  static SystemContextMenuController? _lastShown;

  /// The target [Rect] that was last given to [show].
  ///
  /// Null if [show] has not been called.
  Rect? _lastTargetRect;

  /// The [IOSSystemContextMenuItem]s that were last given to [show].
  ///
  /// Null if [show] has not been called.
  List<IOSSystemContextMenuItemData>? _lastItems;

  /// True when the instance most recently [show]n has been hidden by the
  /// system.
  bool _hiddenBySystem = false;

  /// Indicates whether the system context menu managed by this controller is
  /// currently being displayed to the user.
  @visibleForTesting
  bool get isVisible => this == _lastShown && !_hiddenBySystem;

  /// After calling [dispose], this instance can no longer be used.
  bool _isDisposed = false;

  // Begin SystemContextMenuClient.

  @override
  void handleSystemHide() {
    assert(!_isDisposed);
    assert(isVisible);
    if (_isDisposed || !isVisible) {
      return;
    }
    if (_lastShown == this) {
      _lastShown = null;
    }
    _hiddenBySystem = true;
    onSystemHide?.call();
  }

  // End SystemContextMenuClient.

  /// Shows the system context menu anchored on the given [Rect].
  ///
  /// Currently only supported on iOS 16.0 and later. Check
  /// [MediaQuery.maybeSupportsShowingSystemContextMenu] before calling this.
  ///
  /// The [Rect] represents what the context menu is pointing to. For example,
  /// for some text selection, this would be the selection [Rect].
  ///
  /// Currently this system context menu is bound to text input. Using this
  /// without an active [TextInputConnection] will be a noop.
  ///
  /// There can only be one system context menu visible at a time. Calling this
  /// while another system context menu is already visible will remove the old
  /// menu before showing the new menu.
  ///
  /// See also:
  ///
  ///  * [hide], which hides the menu shown by this method.
  ///  * [MediaQuery.supportsShowingSystemContextMenu], which indicates whether
  ///    this method is supported on the current platform.
  @Deprecated(
    'Use showWithItems instead. '
    'This feature was deprecated after v3.29.0-0.3.pre.',
  )
  Future<void> show(Rect targetRect) {
    assert(!_isDisposed);
    assert(
      TextInput._instance._currentConnection != null,
      'Currently, the system context menu can only be shown for an active text input connection',
    );

    // Don't show the same thing that's already being shown.
    if (_lastShown != null && _lastShown!.isVisible && _lastShown!._lastTargetRect == targetRect) {
      return Future<void>.value();
    }

    assert(
      _lastShown == null || _lastShown == this || !_lastShown!.isVisible,
      'Attempted to show while another instance was still visible.',
    );

    ServicesBinding.systemContextMenuClient = this;

    _lastTargetRect = targetRect;
    _lastShown = this;
    _hiddenBySystem = false;
    return _channel.invokeMethod('ContextMenu.showSystemContextMenu', <String, dynamic>{
      'targetRect': <String, double>{
        'x': targetRect.left,
        'y': targetRect.top,
        'width': targetRect.width,
        'height': targetRect.height,
      },
    });
  }

  /// Shows the system context menu anchored on the given [Rect] with the given
  /// buttons.
  ///
  /// Currently only supported on iOS 16.0 and later. Check
  /// [MediaQuery.maybeSupportsShowingSystemContextMenu] before calling this.
  ///
  /// The [Rect] represents what the context menu is pointing to. For example,
  /// for some text selection, this would be the selection [Rect].
  ///
  /// `items` specifies the buttons that appear in the menu. The buttons that
  /// appear in the menu will be exactly as given and will not automatically
  /// udpate based on the state of the input field. See
  /// [SystemContextMenu.getDefaultItems] for the default items for a given
  /// [EditableTextState].
  ///
  /// Currently this system context menu is bound to text input. Using this
  /// without an active [TextInputConnection] will be a noop.
  ///
  /// There can only be one system context menu visible at a time. Calling this
  /// while another system context menu is already visible will remove the old
  /// menu before showing the new menu.
  ///
  /// See also:
  ///
  ///  * [hide], which hides the menu shown by this method.
  ///  * [MediaQuery.supportsShowingSystemContextMenu], which indicates whether
  ///    this method is supported on the current platform.
  Future<void> showWithItems(Rect targetRect, List<IOSSystemContextMenuItemData> items) {
    assert(!_isDisposed);
    assert(items.isNotEmpty);
    assert(
      TextInput._instance._currentConnection != null,
      'Currently, the system context menu can only be shown for an active text input connection',
    );

    // Don't show the same thing that's already being shown.
    if (_lastShown != null &&
        _lastShown!.isVisible &&
        _lastShown!._lastTargetRect == targetRect &&
        listEquals(_lastShown!._lastItems, items)) {
      return Future<void>.value();
    }

    assert(
      _lastShown == null || _lastShown == this || !_lastShown!.isVisible,
      'Attempted to show while another instance was still visible.',
    );

    ServicesBinding.systemContextMenuClient = this;

    final List<Map<String, dynamic>> itemsJson =
        items.map<Map<String, dynamic>>((IOSSystemContextMenuItemData item) => item._json).toList();
    _lastTargetRect = targetRect;
    _lastItems = items;
    _lastShown = this;
    _hiddenBySystem = false;
    return _channel.invokeMethod('ContextMenu.showSystemContextMenu', <String, dynamic>{
      'targetRect': <String, double>{
        'x': targetRect.left,
        'y': targetRect.top,
        'width': targetRect.width,
        'height': targetRect.height,
      },
      'items': itemsJson,
    });
  }

  /// Hides this system context menu.
  ///
  /// If this hasn't been shown, or if another instance has hidden this menu,
  /// does nothing.
  ///
  /// Currently this is only supported on iOS 16.0 and later.
  ///
  /// See also:
  ///
  ///  * [show], which shows the menu hidden by this method.
  ///  * [MediaQuery.supportsShowingSystemContextMenu], which indicates whether
  ///    the system context menu is supported on the current platform.
  Future<void> hide() async {
    assert(!_isDisposed);
    // This check prevents the instance from accidentally hiding some other
    // instance, since only one can be visible at a time.
    if (this != _lastShown) {
      return;
    }
    _lastShown = null;
    ServicesBinding.systemContextMenuClient = null;
    // This may be called unnecessarily in the case where the user has already
    // hidden the menu (for example by tapping the screen).
    return _channel.invokeMethod<void>('ContextMenu.hideSystemContextMenu');
  }

  @override
  String toString() {
    return 'SystemContextMenuController(onSystemHide=$onSystemHide, _hiddenBySystem=$_hiddenBySystem, _isVisible=$isVisible, _isDisposed=$_isDisposed)';
  }

  /// Used to release resources when this instance will never be used again.
  void dispose() {
    assert(!_isDisposed);
    hide();
    _isDisposed = true;
  }
}

/// Describes a context menu button that will be rendered in the system context
/// menu.
///
/// See also:
///
///  * [SystemContextMenuController], which is used to show the system context
///    menu.
///  * [IOSSystemContextMenuItem], which performs a similar role but at the widget
///    level, where the titles can be replaced with default localized values.
///  * [ContextMenuButtonItem], which performs a similar role for Flutter-drawn
///    context menus.
@immutable
sealed class IOSSystemContextMenuItemData {
  const IOSSystemContextMenuItemData();

  /// The text to display to the user.
  ///
  /// Not exposed for some built-in menu items whose title is always set by the
  /// platform.
  String? get title => null;

  /// The type string used when serialized to json, recognized by the engine.
  String get _jsonType;

  /// Returns json for use in method channel calls, specifically
  /// `ContextMenu.showSystemContextMenu`.
  Map<String, dynamic> get _json {
    return <String, dynamic>{
      'callbackId': hashCode,
      if (title != null) 'title': title,
      'type': _jsonType,
    };
  }

  @override
  int get hashCode => title.hashCode;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is IOSSystemContextMenuItemData && other.title == title;
  }
}

/// A [IOSSystemContextMenuItemData] for the system's built-in copy button.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenuController], which is used to show the system context
///    menu.
///  * [IOSSystemContextMenuItemCopy], which performs a similar role but at the
///    widget level.
final class IOSSystemContextMenuItemDataCopy extends IOSSystemContextMenuItemData {
  /// Creates an instance of [IOSSystemContextMenuItemDataCopy].
  const IOSSystemContextMenuItemDataCopy();

  @override
  String get _jsonType => 'copy';
}

/// A [IOSSystemContextMenuItemData] for the system's built-in cut button.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenuController], which is used to show the system context
///    menu.
///  * [IOSSystemContextMenuItemCut], which performs a similar role but at the
///    widget level.
final class IOSSystemContextMenuItemDataCut extends IOSSystemContextMenuItemData {
  /// Creates an instance of [IOSSystemContextMenuItemDataCut].
  const IOSSystemContextMenuItemDataCut();

  @override
  String get _jsonType => 'cut';
}

/// A [IOSSystemContextMenuItemData] for the system's built-in paste button.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenuController], which is used to show the system context
///    menu.
///  * [IOSSystemContextMenuItemPaste], which performs a similar role but at the
///    widget level.
final class IOSSystemContextMenuItemDataPaste extends IOSSystemContextMenuItemData {
  /// Creates an instance of [IOSSystemContextMenuItemDataPaste].
  const IOSSystemContextMenuItemDataPaste();

  @override
  String get _jsonType => 'paste';
}

/// A [IOSSystemContextMenuItemData] for the system's built-in select all
/// button.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenuController], which is used to show the system context
///    menu.
///  * [IOSSystemContextMenuItemSelectAll], which performs a similar role but at
///    the widget level.
final class IOSSystemContextMenuItemDataSelectAll extends IOSSystemContextMenuItemData {
  /// Creates an instance of [IOSSystemContextMenuItemDataSelectAll].
  const IOSSystemContextMenuItemDataSelectAll();

  @override
  String get _jsonType => 'selectAll';
}

/// A [IOSSystemContextMenuItemData] for the system's built-in look up
/// button.
///
/// Must specify a [title], typically [WidgetsLocalizations.lookUpButtonLabel].
///
/// The action is handled by the platform.
///
/// See also:
///
///  * [SystemContextMenuController], which is used to show the system context
///    menu.
///  * [IOSSystemContextMenuItemLookUp], which performs a similar role but at the
///    widget level, where the title can be replaced with a default localized
///    value.
final class IOSSystemContextMenuItemDataLookUp extends IOSSystemContextMenuItemData {
  /// Creates an instance of [IOSSystemContextMenuItemDataLookUp].
  const IOSSystemContextMenuItemDataLookUp({required this.title});

  @override
  final String title;

  @override
  String get _jsonType => 'lookUp';

  @override
  String toString() {
    return 'IOSSystemContextMenuItemDataLookUp(title: $title)';
  }
}

/// A [IOSSystemContextMenuItemData] for the system's built-in search web
/// button.
///
/// Must specify a [title], typically
/// [WidgetsLocalizations.searchWebButtonLabel].
///
/// The action is handled by the platform.
///
/// See also:
///
///  * [SystemContextMenuController], which is used to show the system context
///    menu.
///  * [IOSSystemContextMenuItemSearchWeb], which performs a similar role but at
///    the widget level, where the title can be replaced with a default localized
///    value.
final class IOSSystemContextMenuItemDataSearchWeb extends IOSSystemContextMenuItemData {
  /// Creates an instance of [IOSSystemContextMenuItemDataSearchWeb].
  const IOSSystemContextMenuItemDataSearchWeb({required this.title});

  @override
  final String title;

  @override
  String get _jsonType => 'searchWeb';

  @override
  String toString() {
    return 'IOSSystemContextMenuItemDataSearchWeb(title: $title)';
  }
}

/// A [IOSSystemContextMenuItemData] for the system's built-in share button.
///
/// Must specify a [title], typically
/// [WidgetsLocalizations.shareButtonLabel].
///
/// The action is handled by the platform.
///
/// See also:
///
///  * [SystemContextMenuController], which is used to show the system context
///    menu.
///  * [IOSSystemContextMenuItemShare], which performs a similar role but at
///    the widget level, where the title can be replaced with a default
///    localized value.
final class IOSSystemContextMenuItemDataShare extends IOSSystemContextMenuItemData {
  /// Creates an instance of [IOSSystemContextMenuItemDataShare].
  const IOSSystemContextMenuItemDataShare({required this.title});

  @override
  final String title;

  @override
  String get _jsonType => 'share';

  @override
  String toString() {
    return 'IOSSystemContextMenuItemDataShare(title: $title)';
  }
}

// TODO(justinmc): Support the "custom" type.
// https://github.com/flutter/flutter/issues/103163
