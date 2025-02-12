// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:typed_data';
///
/// @docImport 'package:flutter/semantics.dart';
/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'binding.dart';
/// @docImport 'clipboard.dart';
/// @docImport 'haptic_feedback.dart';
/// @docImport 'platform_views.dart';
/// @docImport 'raw_keyboard.dart';
/// @docImport 'raw_keyboard_android.dart';
/// @docImport 'raw_keyboard_fuchsia.dart';
/// @docImport 'scribe.dart';
/// @docImport 'system_chrome.dart';
/// @docImport 'system_navigator.dart';
/// @docImport 'system_sound.dart';
/// @docImport 'text_input.dart';
library;

import 'dart:ui';

import 'message_codecs.dart';
import 'platform_channel.dart';

export 'platform_channel.dart' show BasicMessageChannel, MethodChannel;

/// Platform channels used by the Flutter system.
abstract final class SystemChannels {
  /// A JSON [MethodChannel] for navigation.
  ///
  /// The following incoming methods are defined for this channel (registered
  /// using [MethodChannel.setMethodCallHandler]):
  ///
  ///  * `popRoute`, which is called when the system wants the current route to
  ///    be removed (e.g. if the user hits a system-level back button).
  ///
  ///  * `pushRoute`, which is called with a single string argument when the
  ///    operating system instructs the application to open a particular page.
  ///
  ///  * `pushRouteInformation`, which is called with a map, which contains a
  ///    location string and a state object, when the operating system instructs
  ///    the application to open a particular page. These parameters are stored
  ///    under the key `location` and `state` in the map.
  ///
  /// The following methods are used for the opposite direction data flow. The
  /// framework notifies the engine about the route changes.
  ///
  ///  * `selectSingleEntryHistory`, which enables a single-entry history mode.
  ///
  ///  * `selectMultiEntryHistory`, which enables a multiple-entry history mode.
  ///
  ///  * `routeInformationUpdated`, which is called when the application
  ///    navigates to a new location, and which takes two arguments, `location`
  ///    (a URL) and `state` (an object).
  ///
  ///  * `routeUpdated`, a deprecated API which can be called in the same
  ///    situations as `routeInformationUpdated` but whose arguments are
  ///    `routeName` (a URL) and `previousRouteName` (which is ignored).
  ///
  /// These APIs are exposed by the [SystemNavigator] class.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver.didPopRoute] and
  ///    [WidgetsBindingObserver.didPushRoute], which expose this channel's
  ///    methods.
  ///  * [Navigator] which manages transitions from one page to another.
  ///    [Navigator.push], [Navigator.pushReplacement], [Navigator.pop] and
  ///    [Navigator.replace], utilize this channel's methods to send route
  ///    change information from framework to engine.
  static const MethodChannel navigation = OptionalMethodChannel(
    'flutter/navigation',
    JSONMethodCodec(),
  );

  /// A [MethodChannel] for handling predictive back gestures.
  ///
  /// Currently, this feature is only available on Android U and above.
  ///
  /// No outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]).
  ///
  /// The following incoming methods are defined for this channel (registered
  /// using [MethodChannel.setMethodCallHandler]):
  ///
  ///  * `startBackGesture`: The user has started a predictive back gesture.
  ///  * `updateBackGestureProgress`: The user has continued dragging the
  ///    predictive back gesture.
  ///  * `commitBackGesture`: The user has finished a predictive back gesture,
  ///    indicating that the current route should be popped.
  ///  * `cancelBackGesture`: The user has canceled a predictive back gesture,
  ///    indicating that no navigation should occur.
  static const MethodChannel backGesture = OptionalMethodChannel('flutter/backgesture');

  /// A JSON [MethodChannel] for invoking miscellaneous platform methods.
  ///
  /// The following outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `Clipboard.setData`: Places the data from the `text` entry of the
  ///    argument, which must be a [Map], onto the system clipboard. See
  ///    [Clipboard.setData].
  ///
  ///  * `Clipboard.getData`: Returns the data that has the format specified in
  ///    the argument, a [String], from the system clipboard. The only format
  ///    currently supported is `text/plain` ([Clipboard.kTextPlain]). The
  ///    result is a [Map] with a single key, `text`. See [Clipboard.getData].
  ///
  ///  * `HapticFeedback.vibrate`: Triggers a system-default haptic response.
  ///    See [HapticFeedback.vibrate].
  ///
  ///  * `SystemSound.play`: Triggers a system audio effect. The argument must
  ///    be a [String] describing the desired effect; currently only `click` is
  ///    supported. See [SystemSound.play].
  ///
  ///  * `SystemChrome.setPreferredOrientations`: Informs the operating system
  ///    of the desired orientation of the display. The argument is a [List] of
  ///    values which are string representations of values of the
  ///    [DeviceOrientation] enum. See [SystemChrome.setPreferredOrientations].
  ///
  ///  * `SystemChrome.setApplicationSwitcherDescription`: Informs the operating
  ///    system of the desired label and color to be used to describe the
  ///    application in any system-level application lists (e.g. application
  ///    switchers). The argument is a [Map] with two keys, `label` giving a
  ///    [String] description, and `primaryColor` giving a 32 bit integer value
  ///    (the lower eight bits being the blue channel, the next eight bits being
  ///    the green channel, the next eight bits being the red channel, and the
  ///    high eight bits being set, as from [Color.value] for an opaque color).
  ///    The `primaryColor` can also be zero to indicate that the system default
  ///    should be used. See [SystemChrome.setApplicationSwitcherDescription].
  ///
  ///  * `SystemChrome.setEnabledSystemUIOverlays`: Specifies the set of system
  ///    overlays to have visible when the application is running. The argument
  ///    is a [List] of values which are string representations of values of the
  ///    [SystemUiOverlay] enum. See [SystemChrome.setEnabledSystemUIMode].
  ///    [SystemUiOverlay]s can only be configured individually when using
  ///    [SystemUiMode.manual].
  ///
  ///  * `SystemChrome.setEnabledSystemUIMode`: Specifies the [SystemUiMode] for
  ///    the application. The optional `overlays` argument is a [List] of values
  ///    which are string representations of values of the [SystemUiOverlay]
  ///    enum when using [SystemUiMode.manual]. See
  ///    [SystemChrome.setEnabledSystemUIMode].
  ///
  ///  * `SystemChrome.setSystemUIOverlayStyle`: Specifies whether system
  ///    overlays (e.g. the status bar on Android or iOS) should be `light` or
  ///    `dark`. The argument is one of those two strings. See
  ///    [SystemChrome.setSystemUIOverlayStyle].
  ///
  ///  * `SystemNavigator.pop`: Tells the operating system to close the
  ///    application, or the closest equivalent. See [SystemNavigator.pop].
  ///
  ///  * `System.exitApplication`: Tells the engine to send a request back to
  ///    the application to request an application exit (using
  ///    `System.requestAppExit` below), and if it is not canceled, to terminate
  ///    the application using the platform UI toolkit's termination API.
  ///
  /// The following incoming methods are defined for this channel (registered
  /// using [MethodChannel.setMethodCallHandler]):
  ///
  ///  * `SystemChrome.systemUIChange`: The user has changed the visibility of
  ///    the system overlays. This is relevant when using [SystemUiMode]s
  ///    through [SystemChrome.setEnabledSystemUIMode]. See
  ///    [SystemChrome.setSystemUIChangeCallback] to respond to this change in
  ///    application state.
  ///
  ///  * `System.requestAppExit`: The application has requested that it be
  ///    terminated. See [ServicesBinding.exitApplication].
  ///
  ///  * `System.initializationComplete`: Indicate to the engine the
  ///    initialization of a binding that may, among other tasks, register a
  ///    handler for application exit attempts.
  ///
  /// Calls to methods that are not implemented on the shell side are ignored
  /// (so it is safe to call methods when the relevant plugin might be missing).
  static const MethodChannel platform = OptionalMethodChannel(
    'flutter/platform',
    JSONMethodCodec(),
  );

  /// A [MethodChannel] for handling text processing actions.
  ///
  /// This channel exposes the text processing feature for supported platforms.
  /// Currently supported on Android only.
  static const MethodChannel processText = OptionalMethodChannel('flutter/processtext');

  /// A JSON [MethodChannel] for handling text input.
  ///
  /// This channel exposes a system text input control for interacting with IMEs
  /// (input method editors, for example on-screen keyboards). There is one
  /// control, and at any time it can have one active transaction. Transactions
  /// are represented by an integer. New Transactions are started by
  /// `TextInput.setClient`. Messages that are sent are assumed to be for the
  /// current transaction (the last "client" set by `TextInput.setClient`).
  /// Messages received from the shell side specify the transaction to which
  /// they apply, so that stale messages referencing past transactions can be
  /// ignored.
  ///
  /// In debug builds, messages sent with a client ID of -1 are always accepted.
  /// This allows tests to smuggle messages without having to mock the engine's
  /// text handling (for example, allowing the engine to still handle the text
  /// input messages in an integration test).
  ///
  /// The methods described below are wrapped in a more convenient form by the
  /// [TextInput] and [TextInputConnection] class.
  ///
  /// The following outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `TextInput.setClient`: Establishes a new transaction. The arguments is
  ///    a [List] whose first value is an integer representing a previously
  ///    unused transaction identifier, and the second is a [String] with a
  ///    JSON-encoded object with five keys, as obtained from
  ///    [TextInputConfiguration.toJson]. This method must be invoked before any
  ///    others (except `TextInput.hide`). See [TextInput.attach].
  ///
  ///  * `TextInput.show`: Show the keyboard. See [TextInputConnection.show].
  ///
  ///  * `TextInput.setEditingState`: Update the value in the text editing
  ///    control. The argument is a [String] with a JSON-encoded object with
  ///    seven keys, as obtained from [TextEditingValue.toJSON]. See
  ///    [TextInputConnection.setEditingState].
  ///
  ///  * `TextInput.clearClient`: End the current transaction. The next method
  ///    called must be `TextInput.setClient` (or `TextInput.hide`). See
  ///    [TextInputConnection.close].
  ///
  ///  * `TextInput.hide`: Hide the keyboard. Unlike the other methods, this can
  ///    be called at any time. See [TextInputConnection.close].
  ///
  /// The following incoming methods are defined for this channel (registered
  /// using [MethodChannel.setMethodCallHandler]). In each case, the first argument
  /// is a transaction identifier. Calls for stale transactions should be ignored.
  ///
  ///  * `TextInputClient.updateEditingState`: The user has changed the contents
  ///    of the text control. The second argument is an object with seven keys,
  ///    in the form expected by [TextEditingValue.fromJSON].
  ///
  ///  * `TextInputClient.updateEditingStateWithTag`: One or more text controls
  ///    were autofilled by the platform's autofill service. The first argument
  ///    (the client ID) is ignored, the second argument is a map of tags to
  ///    objects in the form expected by [TextEditingValue.fromJSON]. See
  ///    [AutofillScope.getAutofillClient] for details on the interpretation of
  ///    the tag.
  ///
  ///  * `TextInputClient.performAction`: The user has triggered an action. The
  ///    second argument is a [String] consisting of the stringification of one
  ///    of the values of the [TextInputAction] enum.
  ///
  ///  * `TextInputClient.requestExistingInputState`: The embedding may have
  ///    lost its internal state about the current editing client, if there is
  ///    one. The framework should call `TextInput.setClient` and
  ///    `TextInput.setEditingState` again with its most recent information. If
  ///    there is no existing state on the framework side, the call should
  ///    fizzle. (This call is made without a client ID; indeed, without any
  ///    arguments at all.)
  ///
  ///  * `TextInputClient.onConnectionClosed`: The text input connection closed
  ///    on the platform side. For example the application is moved to
  ///    background or used closed the virtual keyboard. This method informs
  ///    [TextInputClient] to clear connection and finalize editing.
  ///    `TextInput.clearClient` and `TextInput.hide` is not called after
  ///    clearing the connection since on the platform side the connection is
  ///    already finalized.
  ///
  /// Calls to methods that are not implemented on the shell side are ignored
  /// (so it is safe to call methods when the relevant plugin might be missing).
  static const MethodChannel textInput = OptionalMethodChannel(
    'flutter/textinput',
    JSONMethodCodec(),
  );

  /// A [MethodChannel] for handling Android Scribe stylus handwriting input.
  ///
  /// Android's Scribe feature allows writing directly on top of a text input
  /// using a stylus.
  ///
  /// The following methods are defined for this channel:
  ///
  ///  * `Scribe.startStylusHandwriting`: Indicates that stylus input has been
  ///  detected and Android should start handwriting input.
  ///  * `Scribe.isStylusHandwritingAvailable`: Returns a boolean representing
  ///  whether or not the device currently supports accepting stylus handwriting
  ///  input. Throws if the device's API level is not sufficient.
  ///  * `Scribe.isFeatureAvailable`: Returns a boolean representing whether or
  ///  not the device currently supports accepting stylus handwriting input.
  ///  Returns false and does not throw if the device's API level is not
  ///  sufficient.
  ///
  /// See also:
  ///
  ///  * [Scribe], which uese this channel.
  ///  * [ScribbleClient], which implements the iOS version of this feature,
  ///    [Scribble](https://support.apple.com/guide/ipad/enter-text-with-scribble-ipad355ab2a7/ipados).
  ///  * <https://developer.android.com/develop/ui/views/touch-and-input/stylus-input/stylus-input-in-text-fields>,
  ///    which is the Android documentation explaining the Scribe feature.
  static const MethodChannel scribe = OptionalMethodChannel('flutter/scribe', JSONMethodCodec());

  /// A [MethodChannel] for handling spell check for text input.
  ///
  /// This channel exposes the spell check framework for supported platforms.
  /// Currently supported on Android and iOS only.
  ///
  /// Spell check requests are initiated by `SpellCheck.initiateSpellCheck`.
  /// These requests may either be completed or canceled. If the request is
  /// completed, the shell side will respond with the results of the request.
  /// Otherwise, the shell side will respond with null.
  ///
  /// The following outgoing methods are defined for this channel (invoked by
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `SpellCheck.initiateSpellCheck`: Sends request for specified text to be
  ///     spell checked and returns the result, either a [List<SuggestionSpan>]
  ///     representing the spell check results of the text or null if the request
  ///     was canceled. The arguments are the [String] to be spell checked
  ///     and the [Locale] for the text to be spell checked with.
  static const MethodChannel spellCheck = OptionalMethodChannel('flutter/spellcheck');

  /// A JSON [MethodChannel] for handling undo events.
  static const MethodChannel undoManager = OptionalMethodChannel(
    'flutter/undomanager',
    JSONMethodCodec(),
  );

  /// A JSON [BasicMessageChannel] for keyboard events.
  ///
  /// Each incoming message received on this channel (registered using
  /// [BasicMessageChannel.setMessageHandler]) consists of a [Map] with
  /// platform-specific data, plus a `type` field which is either `keydown`, or
  /// `keyup`.
  ///
  /// On Android, the available fields are those described by
  /// [RawKeyEventDataAndroid]'s properties.
  ///
  /// On Fuchsia, the available fields are those described by
  /// [RawKeyEventDataFuchsia]'s properties.
  ///
  /// No messages are sent on other platforms currently.
  ///
  /// See also:
  ///
  ///  * [RawKeyboard], which uses this channel to expose key data.
  ///  * [RawKeyEvent.fromMessage], which can decode this data into the [RawKeyEvent]
  ///    subclasses mentioned above.
  static const BasicMessageChannel<Object?> keyEvent = BasicMessageChannel<Object?>(
    'flutter/keyevent',
    JSONMessageCodec(),
  );

  /// A string [BasicMessageChannel] for lifecycle events.
  ///
  /// Valid messages are string representations of the values of the
  /// [AppLifecycleState] enumeration. A handler can be registered using
  /// [BasicMessageChannel.setMessageHandler].
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver.didChangeAppLifecycleState], which triggers
  ///    whenever a message is received on this channel.
  static const BasicMessageChannel<String?> lifecycle = BasicMessageChannel<String?>(
    'flutter/lifecycle',
    StringCodec(),
  );

  /// A JSON [BasicMessageChannel] for system events.
  ///
  /// Events are exposed as [Map]s with string keys. The `type` key specifies
  /// the type of the event; the currently supported system event types are
  /// those listed below. A handler can be registered using
  /// [BasicMessageChannel.setMessageHandler].
  ///
  ///  * `memoryPressure`: Indicates that the operating system would like
  ///    applications to release caches to free up more memory. See
  ///    [WidgetsBindingObserver.didHaveMemoryPressure], which triggers whenever
  ///    a message is received on this channel.
  static const BasicMessageChannel<Object?> system = BasicMessageChannel<Object?>(
    'flutter/system',
    JSONMessageCodec(),
  );

  /// A [BasicMessageChannel] for accessibility events.
  ///
  /// See also:
  ///
  ///  * [SemanticsEvent] and its subclasses for a list of valid accessibility
  ///    events that can be sent over this channel.
  ///  * [SemanticsNode.sendEvent], which uses this channel to dispatch events.
  static const BasicMessageChannel<Object?> accessibility = BasicMessageChannel<Object?>(
    'flutter/accessibility',
    StandardMessageCodec(),
  );

  /// A [MethodChannel] for controlling platform views.
  ///
  /// See also:
  ///
  ///  * [PlatformViewsService] for the available operations on this channel.
  static const MethodChannel platform_views = MethodChannel('flutter/platform_views');

  /// A [MethodChannel] for controlling platform views.
  ///
  /// See also:
  ///
  ///  * [PlatformViewsService] for the available operations on this channel.
  static const MethodChannel platform_views_2 = MethodChannel('flutter/platform_views_2');

  /// A [MethodChannel] for configuring the Skia graphics library.
  ///
  /// The following outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `Skia.setResourceCacheMaxBytes`: Set the maximum number of bytes that
  ///    can be held in the GPU resource cache.
  static const MethodChannel skia = MethodChannel('flutter/skia', JSONMethodCodec());

  /// A [MethodChannel] for configuring mouse cursors.
  ///
  /// All outgoing methods defined for this channel uses a `Map<String, Object?>`
  /// to contain multiple parameters, including the following methods (invoked
  /// using [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `activateSystemCursor`: Request to set the cursor of a pointer
  ///    device to a system cursor. The parameters are
  ///    integer `device`, and string `kind`.
  static const MethodChannel mouseCursor = OptionalMethodChannel('flutter/mousecursor');

  /// A [MethodChannel] for synchronizing restoration data with the engine.
  ///
  /// The following outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `get`: Retrieves the current restoration information (e.g. provided by
  ///    the operating system) from the engine. The method returns a map
  ///    containing an `enabled` boolean to indicate whether collecting
  ///    restoration data is supported by the embedder. If `enabled` is true,
  ///    the map may also contain restoration data stored under the `data` key
  ///    from which the state of the framework may be restored. The restoration
  ///    data is encoded as [Uint8List].
  ///  * `put`: Sends the current restoration data to the engine. Takes the
  ///    restoration data encoded as [Uint8List] as argument.
  ///
  /// The following incoming methods are defined for this channel (registered
  /// using [MethodChannel.setMethodCallHandler]).
  ///
  ///  * `push`: Called by the engine to send newly provided restoration
  ///    information to the framework. The argument given to this method has
  ///    the same format as the object that the `get` method returns.
  ///
  /// See also:
  ///
  ///  * [RestorationManager], which uses this channel and also describes how
  ///    restoration data is used in Flutter.
  static const MethodChannel restoration = OptionalMethodChannel('flutter/restoration');

  /// A [MethodChannel] for installing and managing deferred components.
  ///
  /// The following outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `installDeferredComponent`: Requests that a deferred component identified by
  ///    the provided loadingUnitId or componentName be downloaded and installed.
  ///    Providing a loadingUnitId with null componentName will install a component that
  ///    includes the desired loading unit. If a componentName is provided, then the
  ///    deferred component with the componentName will be installed. This method
  ///    returns a future that will not be completed until the feature is fully installed
  ///    and ready to use. When an error occurs, the future will complete an error.
  ///    Calling `loadLibrary()` on a deferred imported library is equivalent to calling
  ///    this method with a loadingUnitId and null componentName.
  ///  * `uninstallDeferredComponent`:  Requests that a deferred component identified by
  ///    the provided loadingUnitId or componentName be uninstalled. Since
  ///    uninstallation typically requires significant disk i/o, this method only
  ///    signals the intent to uninstall. Actual uninstallation (eg, removal of
  ///    assets and files) may occur at a later time. However, once uninstallation
  ///    is requested, the deferred component should not be used anymore until
  ///    `installDeferredComponent` or `loadLibrary` is called again.
  static const MethodChannel deferredComponent = OptionalMethodChannel('flutter/deferredcomponent');

  /// A JSON [MethodChannel] for localization.
  ///
  /// The following outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `Localization.getStringResource`: Obtains the native string resource
  ///    for a specific locale. The argument is a [Map] with two keys, `key`
  ///    giving a [String] which the resource is defined with, and an optional
  ///    `locale` which is a [String] containing the BCP47 locale identifier of
  ///    the locale requested. See [Locale.toLanguageTag]. When `locale` is not
  ///    specified, the current system locale is used instead.
  static const MethodChannel localization = OptionalMethodChannel(
    'flutter/localization',
    JSONMethodCodec(),
  );

  /// A [MethodChannel] for platform menu specification and control.
  ///
  /// The following outgoing method is defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `Menu.setMenus`: sends the configuration of the platform menu, including
  ///    labels, enable/disable information, and unique integer identifiers for
  ///    each menu item. The configuration is sent as a `Map<String, Object?>`
  ///    encoding the list of top level menu items in window "0", which each
  ///    have a hierarchy of `Map<String, Object?>` containing the required
  ///    data, sent via a [StandardMessageCodec]. It is typically generated from
  ///    a list of [PlatformMenuItem]s, and ends up looking like this example:
  ///
  /// ```dart
  /// Map<String, Object?> menu = <String, Object?>{
  ///   '0': <Map<String, Object?>>[
  ///     <String, Object?>{
  ///       'id': 1,
  ///       'label': 'First Menu Label',
  ///       'enabled': true,
  ///       'children': <Map<String, Object?>>[
  ///         <String, Object?>{
  ///           'id': 2,
  ///           'label': 'Sub Menu Label',
  ///           'enabled': true,
  ///         },
  ///       ],
  ///     },
  ///   ],
  /// };
  /// ```
  ///
  /// The following incoming methods are defined for this channel (registered
  /// using [MethodChannel.setMethodCallHandler]).
  ///
  ///  * `Menu.selectedCallback`: Called when a menu item is selected, along
  ///    with the unique ID of the menu item selected.
  ///
  ///  * `Menu.opened`: Called when a submenu is opened, along with the unique
  ///    ID of the submenu.
  ///
  ///  * `Menu.closed`: Called when a submenu is closed, along with the unique
  ///    ID of the submenu.
  ///
  /// See also:
  ///
  ///  * [DefaultPlatformMenuDelegate], which uses this channel.
  static const MethodChannel menu = OptionalMethodChannel('flutter/menu');

  /// A [MethodChannel] for configuring the browser's context menu on web.
  ///
  /// The following outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `enableContextMenu`: enables the browser's context menu. When a Flutter
  ///    app starts, the browser's context menu is already enabled.
  ///  * `disableContextMenu`: disables the browser's context menu.
  static const MethodChannel contextMenu = OptionalMethodChannel(
    'flutter/contextmenu',
    JSONMethodCodec(),
  );

  /// A [MethodChannel] for retrieving keyboard pressed keys from the engine.
  ///
  /// The following outgoing methods are defined for this channel (invoked using
  /// [OptionalMethodChannel.invokeMethod]):
  ///
  ///  * `getKeyboardState`: Obtains keyboard pressed keys from the engine.
  ///    The keyboard state is sent as a `Map<int, int>?` where each entry
  ///    represents a pressed keyboard key. The entry key is the physical
  ///    key ID and the entry value is the logical key ID.
  ///
  ///    Both the framework and the engine maintain a state of the current
  ///    pressed keys. There are edge cases, related to startup and restart,
  ///    where the framework needs to resynchronize its keyboard state.
  ///
  /// See also:
  ///
  ///  * [HardwareKeyboard.syncKeyboardState], which uses this channel to synchronize
  ///    the `HardwareKeyboard` pressed state.
  static const MethodChannel keyboard = OptionalMethodChannel('flutter/keyboard');
}
