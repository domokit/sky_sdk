// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'binding.dart';
library;

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';

import 'message_codecs.dart';
import 'system_channels.dart';

export 'dart:typed_data' show Uint8List;

typedef _BucketVisitor = void Function(RestorationBucket bucket);

/// Manages the restoration data in the framework and synchronizes it with the
/// engine.
///
/// Restoration data can be serialized out and - at a later point in time - be
/// used to restore the application to the previous state described by the
/// serialized data. Mobile operating systems use the concept of state
/// restoration to provide the illusion that apps continue to run in the
/// background forever: after an app has been backgrounded, the user can always
/// return to it and find it in the same state. In practice, the operating
/// system may, however, terminate the app to free resources for other apps
/// running in the foreground. Before that happens, the app gets a chance to
/// serialize out its restoration data. When the user navigates back to the
/// backgrounded app, it is restarted and the serialized restoration data is
/// provided to it again. Ideally, the app will use that data to restore itself
/// to the same state it was in when the user backgrounded the app.
///
/// In Flutter, restoration data is organized in a tree of [RestorationBucket]s
/// which is rooted in the [rootBucket]. All information that the application
/// needs to restore its current state must be stored in a bucket in this
/// hierarchy. To store data in the hierarchy, entities (e.g. [Widget]s) must
/// claim ownership of a child bucket from a parent bucket (which may be the
/// [rootBucket] provided by this [RestorationManager]). The owner of a bucket
/// may store arbitrary values in the bucket as long as they can be serialized
/// with the [StandardMessageCodec]. The values are stored in the bucket under a
/// given restoration ID as key. A restoration ID is a [String] that must be
/// unique within a given bucket. To access the stored value again during state
/// restoration, the same restoration ID must be provided again. The owner of
/// the bucket may also make the bucket available to other entities so that they
/// can claim child buckets from it for their own restoration needs. Within a
/// bucket, child buckets are also identified by unique restoration IDs. The
/// restoration ID must be provided when claiming a child bucket.
///
/// When restoration data is provided to the [RestorationManager] (e.g. after
/// the application relaunched when foregrounded again), the bucket hierarchy
/// with all the data stored in it is restored. Entities can retrieve the data
/// again by using the same restoration IDs that they originally used to store
/// the data.
///
/// In addition to providing restoration data when the app is launched,
/// restoration data may also be provided to a running app to restore it to a
/// previous state (e.g. when the user hits the back/forward button in the web
/// browser). When this happens, the [RestorationManager] notifies its listeners
/// (added via [addListener]) that a new [rootBucket] is available. In response
/// to the notification, listeners must stop using the old bucket and restore
/// their state from the information in the new [rootBucket].
///
/// Some platforms restrict the size of the restoration data. Therefore, the
/// data stored in the buckets should be as small as possible while still
/// allowing the app to restore its current state from it. Data that can be
/// retrieved from other services (e.g. a database or a web server) should not
/// be included in the restoration data. Instead, a small identifier (e.g. a
/// UUID, database record number, or resource locator) should be stored that can
/// be used to retrieve the data again from its original source during state
/// restoration.
///
/// The [RestorationManager] sends a serialized version of the bucket hierarchy
/// over to the engine at the end of a frame in which the data in the hierarchy
/// or its shape has changed. The engine caches the data until the operating
/// system needs it. The application is responsible for keeping the data in the
/// bucket always up-to-date to reflect its current state.
///
/// ## Discussion
///
/// Due to Flutter's threading model and restrictions in the APIs of the
/// platforms Flutter runs on, restoration data must be stored in the buckets
/// proactively as described above. When the operating system asks for the
/// restoration data, it will do so on the platform thread expecting a
/// synchronous response. To avoid the risk of deadlocks, the platform thread
/// cannot block and call into the UI thread (where the dart code is running) to
/// retrieve the restoration data. For this reason, the [RestorationManager]
/// always sends the latest copy of the restoration data from the UI thread over
/// to the platform thread whenever it changes. That way, the restoration data
/// is always ready to go on the platform thread when the operating system needs
/// it.
///
/// ## State Restoration on iOS
///
/// To enable state restoration on iOS, a restoration identifier has to be
/// assigned to the [FlutterViewController](/ios-embedder/interface_flutter_view_controller.html).
/// If the standard embedding (produced by `flutter create`) is used, this can
/// be accomplished with the following steps:
///
///  1. In the app's directory, open `ios/Runner.xcodeproj` with Xcode.
///  2. Select `Main.storyboard` under `Runner/Runner` in the Project Navigator
///     on the left.
///  3. Select the `Flutter View Controller` under
///     `Flutter View Controller Scene` in the view hierarchy.
///  4. Navigate to the Identity Inspector in the panel on the right.
///  5. Enter a unique restoration ID in the provided field.
///  6. Save the project.
///
/// ## Development with hot restart and hot reload
///
/// Changes applied to your app with hot reload and hot restart are not
/// persisted on the device. They are lost when the app is fully terminated and
/// restarted, e.g. by the operating system. Therefore, your app may not restore
/// correctly during development if you have made changes and applied them with
/// hot restart or hot reload. To test state restoration, always make sure to
/// fully re-compile your application (e.g. by re-executing `flutter run`) after
/// making a change.
///
/// ## Testing State Restoration
///
/// {@template flutter.widgets.RestorationManager}
/// To test state restoration on Android:
///   1. Turn on "Don't keep activities", which destroys the Android activity
///      as soon as the user leaves it. This option should become available
///      when Developer Options are turned on for the device.
///   2. Run the code sample on an Android device.
///   3. Create some in-memory state in the app on the phone,
///      e.g. by navigating to a different screen.
///   4. Background the Flutter app, then return to it. It will restart
///      and restore its state.
///
/// To test state restoration on iOS:
///   1. Open `ios/Runner.xcworkspace/` in Xcode.
///   2. (iOS 14+ only): Switch to build in profile or release mode, as
///      launching an app from the home screen is not supported in debug
///      mode.
///   2. Press the Play button in Xcode to build and run the app.
///   3. Create some in-memory state in the app on the phone,
///      e.g. by navigating to a different screen.
///   4. Background the app on the phone, e.g. by going back to the home screen.
///   5. Press the Stop button in Xcode to terminate the app while running in
///      the background.
///   6. Open the app again on the phone (not via Xcode). It will restart
///      and restore its state.
/// {@endtemplate}
///
/// See also:
///
///  * [ServicesBinding.restorationManager], which holds the singleton instance
///    of the [RestorationManager] for the currently running application.
///  * [RestorationBucket], which make up the restoration data hierarchy.
///  * [RestorationMixin], which uses [RestorationBucket]s behind the scenes
///    to make [State] objects of [StatefulWidget]s restorable.
class RestorationManager extends ChangeNotifier {
  /// Construct the restoration manager and set up the communications channels
  /// with the engine to get restoration messages (by calling [initChannels]).
  RestorationManager() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
    initChannels();
  }

  /// Sets up the method call handler for [SystemChannels.restoration].
  ///
  /// This is called by the constructor to configure the communications channel
  /// with the Flutter engine to get restoration messages.
  ///
  /// Subclasses (especially in tests) can override this to avoid setting up
  /// that communications channel, or to set it up differently, as necessary.
  @protected
  void initChannels() {
    SystemChannels.restoration.setMethodCallHandler(_methodHandler);
  }

  /// The root of the [RestorationBucket] hierarchy containing the restoration
  /// data.
  ///
  /// Child buckets can be claimed from this bucket via
  /// [RestorationBucket.claimChild]. If the [RestorationManager] has been asked
  /// to restore the application to a previous state, these buckets will contain
  /// the previously stored data. Otherwise the root bucket (and all children
  /// claimed from it) will be empty.
  ///
  /// The [RestorationManager] informs its listeners (added via [addListener])
  /// when the value returned by this getter changes. This happens when new
  /// restoration data has been provided to the [RestorationManager] to restore
  /// the application to a different state. In response to the notification,
  /// listeners must stop using the old root bucket and obtain the new one via
  /// this getter ([rootBucket] will have been updated to return the new bucket
  /// just before the listeners are notified).
  ///
  /// The restoration data describing the current bucket hierarchy is retrieved
  /// asynchronously from the engine the first time the root bucket is accessed
  /// via this getter. After the data has been copied over from the engine, this
  /// getter will return a [SynchronousFuture], that immediately resolves to the
  /// root [RestorationBucket].
  ///
  /// The returned [Future] may resolve to null if state restoration is
  /// currently turned off.
  ///
  /// See also:
  ///
  ///  * [RootRestorationScope], which makes the root bucket available in the
  ///    [Widget] tree.
  Future<RestorationBucket?> get rootBucket {
    if (_rootBucketIsValid) {
      return SynchronousFuture<RestorationBucket?>(_rootBucket);
    }
    if (_pendingRootBucket == null) {
      _pendingRootBucket = Completer<RestorationBucket?>();
      _getRootBucketFromEngine();
    }
    return _pendingRootBucket!.future;
  }

  RestorationBucket? _rootBucket; // May be null to indicate that restoration is turned off.
  Completer<RestorationBucket?>? _pendingRootBucket;
  bool _rootBucketIsValid = false;

  /// Returns true for the frame after [rootBucket] has been replaced with a
  /// new non-null bucket.
  ///
  /// When true, entities should forget their current state and restore
  /// their state according to the information in the new [rootBucket].
  ///
  /// The [RestorationManager] informs its listeners (added via [addListener])
  /// when this flag changes from false to true.
  bool get isReplacing => _isReplacing;
  bool _isReplacing = false;

  Future<void> _getRootBucketFromEngine() async {
    final Map<Object?, Object?>? config = await SystemChannels.restoration
        .invokeMethod<Map<Object?, Object?>>('get');
    if (_pendingRootBucket == null) {
      // The restoration data was obtained via other means (e.g. by calling
      // [handleRestorationDataUpdate] while the request to the engine was
      // outstanding. Ignore the engine's response.
      return;
    }
    assert(_rootBucket == null);
    _parseAndHandleRestorationUpdateFromEngine(config);
  }

  void _parseAndHandleRestorationUpdateFromEngine(Map<Object?, Object?>? update) {
    handleRestorationUpdateFromEngine(
      enabled: update != null && update['enabled']! as bool,
      data: update == null ? null : update['data'] as Uint8List?,
    );
  }

  /// Called by the [RestorationManager] on itself to parse the restoration
  /// information obtained from the engine.
  ///
  /// The `enabled` parameter indicates whether the engine wants to receive
  /// restoration data. When `enabled` is false, state restoration is turned
  /// off and the [rootBucket] is set to null. When `enabled` is true, the
  /// provided restoration `data` will be parsed into a new [rootBucket]. If
  /// `data` is null, an empty [rootBucket] will be instantiated.
  ///
  /// Subclasses in test frameworks may call this method at any time to inject
  /// restoration data (obtained e.g. by overriding [sendToEngine]) into the
  /// [RestorationManager]. When the method is called before the [rootBucket] is
  /// accessed, [rootBucket] will complete synchronously the next time it is
  /// called.
  @protected
  void handleRestorationUpdateFromEngine({required bool enabled, required Uint8List? data}) {
    assert(enabled || data == null);

    _isReplacing = _rootBucketIsValid && enabled;
    if (_isReplacing) {
      SchedulerBinding.instance.addPostFrameCallback((Duration _) {
        _isReplacing = false;
      }, debugLabel: 'RestorationManager.resetIsReplacing');
    }

    final RestorationBucket? oldRoot = _rootBucket;
    _rootBucket =
        enabled
            ? RestorationBucket.root(manager: this, rawData: _decodeRestorationData(data))
            : null;
    _rootBucketIsValid = true;
    assert(_pendingRootBucket == null || !_pendingRootBucket!.isCompleted);
    _pendingRootBucket?.complete(_rootBucket);
    _pendingRootBucket = null;

    if (_rootBucket != oldRoot) {
      notifyListeners();
      oldRoot?.dispose();
    }
  }

  /// Called by the [RestorationManager] on itself to send the provided
  /// encoded restoration data to the engine.
  ///
  /// The `encodedData` describes the entire bucket hierarchy that makes up the
  /// current restoration data.
  ///
  /// Subclasses in test frameworks may override this method to capture the
  /// restoration data that would have been send to the engine. The captured
  /// data can be re-injected into the [RestorationManager] via the
  /// [handleRestorationUpdateFromEngine] method to restore the state described
  /// by the data.
  @protected
  Future<void> sendToEngine(Uint8List encodedData) {
    return SystemChannels.restoration.invokeMethod<void>('put', encodedData);
  }

  Future<void> _methodHandler(MethodCall call) async {
    switch (call.method) {
      case 'push':
        _parseAndHandleRestorationUpdateFromEngine(call.arguments as Map<Object?, Object?>);
      default:
        throw UnimplementedError(
          "${call.method} was invoked but isn't implemented by $runtimeType",
        );
    }
  }

  Map<Object?, Object?>? _decodeRestorationData(Uint8List? data) {
    if (data == null) {
      return null;
    }
    final ByteData encoded = data.buffer.asByteData(data.offsetInBytes, data.lengthInBytes);
    return const StandardMessageCodec().decodeMessage(encoded) as Map<Object?, Object?>?;
  }

  Uint8List _encodeRestorationData(Map<Object?, Object?> data) {
    final ByteData encoded = const StandardMessageCodec().encodeMessage(data)!;
    return encoded.buffer.asUint8List(encoded.offsetInBytes, encoded.lengthInBytes);
  }

  bool _debugDoingUpdate = false;
  bool _serializationScheduled = false;

  final Set<RestorationBucket> _bucketsNeedingSerialization = <RestorationBucket>{};

  /// Called by a [RestorationBucket] to request serialization for that bucket.
  ///
  /// This method is called by a bucket in the hierarchy whenever the data
  /// in it or the shape of the hierarchy has changed.
  ///
  /// Calling this is a no-op when the bucket is already scheduled for
  /// serialization.
  ///
  /// It is exposed to allow testing of [RestorationBucket]s in isolation.
  @protected
  @visibleForTesting
  void scheduleSerializationFor(RestorationBucket bucket) {
    assert(bucket._manager == this);
    assert(!_debugDoingUpdate);
    _bucketsNeedingSerialization.add(bucket);
    if (!_serializationScheduled) {
      _serializationScheduled = true;
      SchedulerBinding.instance.addPostFrameCallback(
        (Duration _) => _doSerialization(),
        debugLabel: 'RestorationManager.doSerialization',
      );
    }
  }

  /// Called by a [RestorationBucket] to unschedule a request for serialization.
  ///
  /// This method is called by a bucket in the hierarchy whenever it no longer
  /// needs to be serialized (e.g. because the bucket got disposed).
  ///
  /// It is safe to call this even when the bucket wasn't scheduled for
  /// serialization before.
  ///
  /// It is exposed to allow testing of [RestorationBucket]s in isolation.
  @protected
  @visibleForTesting
  void unscheduleSerializationFor(RestorationBucket bucket) {
    assert(bucket._manager == this);
    assert(!_debugDoingUpdate);
    _bucketsNeedingSerialization.remove(bucket);
  }

  void _doSerialization() {
    if (!_serializationScheduled) {
      return;
    }
    assert(() {
      _debugDoingUpdate = true;
      return true;
    }());
    _serializationScheduled = false;

    for (final RestorationBucket bucket in _bucketsNeedingSerialization) {
      bucket.finalize();
    }
    _bucketsNeedingSerialization.clear();
    sendToEngine(_encodeRestorationData(_rootBucket!._rawData));

    assert(() {
      _debugDoingUpdate = false;
      return true;
    }());
  }

  /// Called to manually flush the restoration data to the engine.
  ///
  /// A change in restoration data is usually accompanied by scheduling a frame
  /// (because the restoration data is modified inside a [State.setState] call,
  /// because it is usually something that affects the interface). Restoration
  /// data is automatically flushed to the engine at the end of a frame. As a
  /// result, it is uncommon to need to call this method directly. However, if
  /// restoration data is changed without triggering a frame, this method must
  /// be called to ensure that the updated restoration data is sent to the
  /// engine in a timely manner. An example of such a use case is the
  /// [Scrollable], where the final scroll offset after a scroll activity
  /// finishes is determined between frames without scheduling a new frame.
  ///
  /// Calling this method is a no-op if a frame is already scheduled. In that
  /// case, the restoration data will be flushed to the engine at the end of
  /// that frame. If this method is called and no frame is scheduled, the
  /// current restoration data is directly sent to the engine.
  void flushData() {
    assert(!_debugDoingUpdate);
    if (SchedulerBinding.instance.hasScheduledFrame) {
      return;
    }
    _doSerialization();
    assert(!_serializationScheduled);
  }

  @override
  void dispose() {
    _rootBucket?.dispose();
    super.dispose();
  }
}

/// A [RestorationBucket] holds pieces of the restoration data that a part of
/// the application needs to restore its state.
///
/// For a general overview of how state restoration works in Flutter, see the
/// [RestorationManager].
///
/// [RestorationBucket]s are organized in a tree that is rooted in
/// [RestorationManager.rootBucket] and managed by a [RestorationManager]. The
/// tree is serializable and must contain all the data an application needs to
/// restore its current state at a later point in time.
///
/// A [RestorationBucket] stores restoration data as key-value pairs. The key is
/// a [String] representing a restoration ID that identifies a piece of data
/// uniquely within a bucket. The value can be anything that is serializable via
/// the [StandardMessageCodec]. Furthermore, a [RestorationBucket] may have
/// child buckets, which are identified within their parent via a unique
/// restoration ID as well.
///
/// During state restoration, the data previously stored in the
/// [RestorationBucket] hierarchy will be made available again to the
/// application to restore it to the state it had when the data was collected.
/// State restoration to a previous state may happen when the app is launched
/// (e.g. after it has been terminated gracefully while running in the
/// background) or after the app has already been running for a while.
///
/// ## Lifecycle
///
/// A [RestorationBucket] is rarely instantiated directly via its constructors.
/// Instead, when an entity wants to store data in or retrieve data from a
/// restoration bucket, it typically obtains a child bucket from a parent by
/// calling [claimChild]. If no parent is available,
/// [RestorationManager.rootBucket] may be used as a parent. When claiming a
/// child, the claimer must provide the restoration ID of the child it would
/// like to own. A child bucket with a given restoration ID can at most have
/// one owner. If another owner tries to claim a bucket with the same ID from
/// the same parent, an exception is thrown (see discussion in [claimChild]).
/// The restoration IDs that a given owner uses to claim a child (and to store
/// data in that child, see below) must be stable across app launches to ensure
/// that after the app restarts the owner can retrieve the same data again that
/// it stored during a previous run.
///
/// Per convention, the owner of the bucket has exclusive access to the values
/// stored in the bucket. It can read, add, modify, and remove values via the
/// [read], [write], and [remove] methods. In general, the owner should store
/// all the data in the bucket that it needs to restore its current state. If
/// its current state changes, the data in the bucket must be updated. At the
/// same time, the data in the bucket should be kept to a minimum. For example,
/// for data that can be retrieved from other sources (like a database or
/// web service) only enough information (e.g. an ID or resource locator) to
/// re-obtain that data should be stored in the bucket. In addition to managing
/// the data in a bucket, an owner may also make the bucket available to other
/// entities so they can claim child buckets from it via [claimChild] for their
/// own restoration needs.
///
/// The bucket returned by [claimChild] may either contain state information
/// that the owner had previously (e.g. during a previous run of the
/// application) stored in it or it may be empty. If the bucket contains data,
/// the owner is expected to restore its state with the information previously
/// stored in the bucket. If the bucket is empty, it may initialize itself to
/// default values.
///
/// When the data stored in a bucket is no longer needed to restore the
/// application to its current state (e.g. because the owner of the bucket is no
/// longer shown on screen), the bucket must be [dispose]d. This will remove all
/// information stored in the bucket from the app's restoration data and that
/// information will not be available again when the application is restored to
/// this state in the future.
class RestorationBucket {
  /// Creates an empty [RestorationBucket] to be provided to [adoptChild] to add
  /// it to the bucket hierarchy.
  ///
  /// {@template flutter.services.RestorationBucket.empty.bucketCreation}
  /// Instantiating a bucket directly is rare, most buckets are created by
  /// claiming a child from a parent via [claimChild]. If no parent bucket is
  /// available, [RestorationManager.rootBucket] may be used as a parent.
  /// {@endtemplate}
  RestorationBucket.empty({required String restorationId, required Object? debugOwner})
    : _restorationId = restorationId,
      _rawData = <String, Object?>{} {
    assert(() {
      _debugOwner = debugOwner;
      return true;
    }());
    if (kFlutterMemoryAllocationsEnabled) {
      _maybeDispatchObjectCreation();
    }
  }

  /// Creates the root [RestorationBucket] for the provided restoration
  /// `manager`.
  ///
  /// The `rawData` must either be null (in which case an empty bucket will be
  /// instantiated) or it must be a nested map describing the entire bucket
  /// hierarchy in the following format:
  ///
  /// ```javascript
  /// {
  ///  'v': {  // key-value pairs
  ///     // * key is a string representation a restoration ID
  ///     // * value is any primitive that can be encoded with [StandardMessageCodec]
  ///    '<restoration-id>: <Object>,
  ///   },
  ///  'c': {  // child buckets
  ///    'restoration-id': <nested map representing a child bucket>
  ///   }
  /// }
  /// ```
  ///
  /// {@macro flutter.services.RestorationBucket.empty.bucketCreation}
  RestorationBucket.root({
    required RestorationManager manager,
    required Map<Object?, Object?>? rawData,
  }) : _manager = manager,
       _rawData = rawData ?? <Object?, Object?>{},
       _restorationId = 'root' {
    assert(() {
      _debugOwner = manager;
      return true;
    }());
    if (kFlutterMemoryAllocationsEnabled) {
      _maybeDispatchObjectCreation();
    }
  }

  /// Creates a child bucket initialized with the data that the provided
  /// `parent` has stored under the provided [restorationId].
  ///
  /// This constructor cannot be used if the `parent` does not have any child
  /// data stored under the given ID. In that case, create an empty bucket (via
  /// [RestorationBucket.empty] and have the parent adopt it via [adoptChild].
  ///
  /// {@macro flutter.services.RestorationBucket.empty.bucketCreation}
  RestorationBucket.child({
    required String restorationId,
    required RestorationBucket parent,
    required Object? debugOwner,
  }) : assert(parent._rawChildren[restorationId] != null),
       _manager = parent._manager,
       _parent = parent,
       _rawData = parent._rawChildren[restorationId]! as Map<Object?, Object?>,
       _restorationId = restorationId {
    assert(() {
      _debugOwner = debugOwner;
      return true;
    }());
    if (kFlutterMemoryAllocationsEnabled) {
      _maybeDispatchObjectCreation();
    }
  }

  static const String _childrenMapKey = 'c';
  static const String _valuesMapKey = 'v';

  final Map<Object?, Object?> _rawData;

  /// The owner of the bucket that was provided when the bucket was claimed via
  /// [claimChild].
  ///
  /// The value is used in error messages. Accessing the value is only valid
  /// in debug mode, otherwise it will return null.
  Object? get debugOwner {
    assert(_debugAssertNotDisposed());
    return _debugOwner;
  }

  Object? _debugOwner;

  RestorationManager? _manager;
  RestorationBucket? _parent;

  /// Returns true when entities processing this bucket should restore their
  /// state from the information in the bucket (e.g. via [read] and
  /// [claimChild]) instead of copying their current state information into the
  /// bucket (e.g. via [write] and [adoptChild].
  ///
  /// This flag is true for the frame after the [RestorationManager] has been
  /// instructed to restore the application from newly provided restoration
  /// data.
  bool get isReplacing => _manager?.isReplacing ?? false;

  /// The restoration ID under which the bucket is currently stored in the
  /// parent of this bucket (or wants to be stored if it is currently
  /// parent-less).
  ///
  /// This value is never null.
  String get restorationId {
    assert(_debugAssertNotDisposed());
    return _restorationId;
  }

  String _restorationId;

  // Maps a restoration ID to the raw map representation of a child bucket.
  Map<Object?, Object?> get _rawChildren =>
      _rawData.putIfAbsent(_childrenMapKey, () => <Object?, Object?>{})! as Map<Object?, Object?>;
  // Maps a restoration ID to a value that is stored in this bucket.
  Map<Object?, Object?> get _rawValues =>
      _rawData.putIfAbsent(_valuesMapKey, () => <Object?, Object?>{})! as Map<Object?, Object?>;

  // Get and store values.

  /// Returns the value of type `P` that is currently stored in the bucket under
  /// the provided `restorationId`.
  ///
  /// Returns null if nothing is stored under that id. Throws, if the value
  /// stored under the ID is not of type `P`.
  ///
  /// See also:
  ///
  ///  * [write], which stores a value in the bucket.
  ///  * [remove], which removes a value from the bucket.
  ///  * [contains], which checks whether any value is stored under a given
  ///    restoration ID.
  P? read<P>(String restorationId) {
    assert(_debugAssertNotDisposed());
    return _rawValues[restorationId] as P?;
  }

  /// Stores the provided `value` of type `P` under the provided `restorationId`
  /// in the bucket.
  ///
  /// Any value that has previously been stored under that ID is overwritten
  /// with the new value. The provided `value` must be serializable with the
  /// [StandardMessageCodec].
  ///
  /// Null values will be stored in the bucket as-is. To remove a value, use
  /// [remove].
  ///
  /// See also:
  ///
  ///  * [read], which retrieves a stored value from the bucket.
  ///  * [remove], which removes a value from the bucket.
  ///  * [contains], which checks whether any value is stored under a given
  ///    restoration ID.
  void write<P>(String restorationId, P value) {
    assert(_debugAssertNotDisposed());
    assert(debugIsSerializableForRestoration(value));
    if (_rawValues[restorationId] != value || !_rawValues.containsKey(restorationId)) {
      _rawValues[restorationId] = value;
      _markNeedsSerialization();
    }
  }

  /// Deletes the value currently stored under the provided `restorationId` from
  /// the bucket.
  ///
  /// The value removed from the bucket is casted to `P` and returned. If no
  /// value was stored under that id, null is returned.
  ///
  /// See also:
  ///
  ///  * [read], which retrieves a stored value from the bucket.
  ///  * [write], which stores a value in the bucket.
  ///  * [contains], which checks whether any value is stored under a given
  ///    restoration ID.
  P? remove<P>(String restorationId) {
    assert(_debugAssertNotDisposed());
    final bool needsUpdate = _rawValues.containsKey(restorationId);
    final P? result = _rawValues.remove(restorationId) as P?;
    if (_rawValues.isEmpty) {
      _rawData.remove(_valuesMapKey);
    }
    if (needsUpdate) {
      _markNeedsSerialization();
    }
    return result;
  }

  /// Checks whether a value stored in the bucket under the provided
  /// `restorationId`.
  ///
  /// See also:
  ///
  ///  * [read], which retrieves a stored value from the bucket.
  ///  * [write], which stores a value in the bucket.
  ///  * [remove], which removes a value from the bucket.
  bool contains(String restorationId) {
    assert(_debugAssertNotDisposed());
    return _rawValues.containsKey(restorationId);
  }

  // Child management.

  // The restoration IDs and associated buckets of children that have been
  // claimed via [claimChild].
  final Map<String, RestorationBucket> _claimedChildren = <String, RestorationBucket>{};
  // Newly created child buckets whose restoration ID is still in use, see
  // comment in [claimChild] for details.
  final Map<String, List<RestorationBucket>> _childrenToAdd = <String, List<RestorationBucket>>{};

  /// Claims ownership of the child with the provided `restorationId` from this
  /// bucket.
  ///
  /// If the application is getting restored to a previous state, the bucket
  /// will contain all the data that was previously stored in the bucket.
  /// Otherwise, an empty bucket is returned.
  ///
  /// The claimer of the bucket is expected to use the data stored in the bucket
  /// to restore itself to its previous state described by the data in the
  /// bucket. If the bucket is empty, it should initialize itself to default
  /// values. Whenever the information that the claimer needs to restore its
  /// state changes, the data in the bucket should be updated to reflect that.
  ///
  /// A child bucket with a given `restorationId` can only have one owner. If
  /// another owner claims a child bucket with the same `restorationId` an
  /// exception will be thrown at the end of the current frame unless the
  /// previous owner has either deleted its bucket by calling [dispose] or has
  /// moved it to a new parent via [adoptChild].
  ///
  /// When the returned bucket is no longer needed, it must be [dispose]d to
  /// delete the information stored in it from the app's restoration data.
  RestorationBucket claimChild(String restorationId, {required Object? debugOwner}) {
    assert(_debugAssertNotDisposed());
    // There are three cases to consider:
    // 1. Claiming an ID that has already been claimed.
    // 2. Claiming an ID that doesn't yet exist in [_rawChildren].
    // 3. Claiming an ID that does exist in [_rawChildren] and hasn't been
    //    claimed yet.
    // If an ID has already been claimed (case 1) the current owner may give up
    // that ID later this frame and it can be re-used. In anticipation of the
    // previous owner's surrender of the id, we return an empty bucket for this
    // new claim and check in [_debugAssertIntegrity] that at the end of the
    // frame the old owner actually did surrendered the id.
    // Case 2 also requires the creation of a new empty bucket.
    // In Case 3 we create a new bucket wrapping the existing data in
    // [_rawChildren].

    // Case 1+2: Adopt and return an empty bucket.
    if (_claimedChildren.containsKey(restorationId) || !_rawChildren.containsKey(restorationId)) {
      final RestorationBucket child = RestorationBucket.empty(
        debugOwner: debugOwner,
        restorationId: restorationId,
      );
      adoptChild(child);
      return child;
    }

    // Case 3: Return bucket wrapping the existing data.
    assert(_rawChildren[restorationId] != null);
    final RestorationBucket child = RestorationBucket.child(
      restorationId: restorationId,
      parent: this,
      debugOwner: debugOwner,
    );
    _claimedChildren[restorationId] = child;
    return child;
  }

  /// Adopts the provided `child` bucket.
  ///
  /// The `child` will be dropped from its old parent, if it had one.
  ///
  /// The `child` is stored under its [restorationId] in this bucket. If this
  /// bucket already contains a child bucket under the same id, the owner of
  /// that existing bucket must give it up (e.g. by moving the child bucket to a
  /// different parent or by disposing it) before the end of the current frame.
  /// Otherwise an exception indicating the illegal use of duplicated
  /// restoration IDs will trigger in debug mode.
  ///
  /// No-op if the provided bucket is already a child of this bucket.
  void adoptChild(RestorationBucket child) {
    assert(_debugAssertNotDisposed());
    if (child._parent != this) {
      child._parent?._removeChildData(child);
      child._parent = this;
      _addChildData(child);
      if (child._manager != _manager) {
        _recursivelyUpdateManager(child);
      }
    }
    assert(child._parent == this);
    assert(child._manager == _manager);
  }

  void _dropChild(RestorationBucket child) {
    assert(child._parent == this);
    _removeChildData(child);
    child._parent = null;
    if (child._manager != null) {
      child._updateManager(null);
      child._visitChildren(_recursivelyUpdateManager);
    }
  }

  bool _needsSerialization = false;
  void _markNeedsSerialization() {
    if (!_needsSerialization) {
      _needsSerialization = true;
      _manager?.scheduleSerializationFor(this);
    }
  }

  /// Called by the [RestorationManager] just before the data of the bucket
  /// is serialized and send to the engine.
  ///
  /// It is exposed to allow testing of [RestorationBucket]s in isolation.
  @visibleForTesting
  void finalize() {
    assert(_debugAssertNotDisposed());
    assert(_needsSerialization);
    _needsSerialization = false;
    assert(_debugAssertIntegrity());
  }

  void _recursivelyUpdateManager(RestorationBucket bucket) {
    bucket._updateManager(_manager);
    bucket._visitChildren(_recursivelyUpdateManager);
  }

  void _updateManager(RestorationManager? newManager) {
    if (_manager == newManager) {
      return;
    }
    if (_needsSerialization) {
      _manager?.unscheduleSerializationFor(this);
    }
    _manager = newManager;
    if (_needsSerialization && _manager != null) {
      _needsSerialization = false;
      _markNeedsSerialization();
    }
  }

  bool _debugAssertIntegrity() {
    assert(() {
      if (_childrenToAdd.isEmpty) {
        return true;
      }
      final List<DiagnosticsNode> error = <DiagnosticsNode>[
        ErrorSummary('Multiple owners claimed child RestorationBuckets with the same IDs.'),
        ErrorDescription('The following IDs were claimed multiple times from the parent $this:'),
      ];
      for (final MapEntry<String, List<RestorationBucket>> child in _childrenToAdd.entries) {
        final String id = child.key;
        final List<RestorationBucket> buckets = child.value;
        assert(buckets.isNotEmpty);
        assert(_claimedChildren.containsKey(id));
        error.addAll(<DiagnosticsNode>[
          ErrorDescription(' * "$id" was claimed by:'),
          ...buckets.map(
            (RestorationBucket bucket) => ErrorDescription('   * ${bucket.debugOwner}'),
          ),
          ErrorDescription('   * ${_claimedChildren[id]!.debugOwner} (current owner)'),
        ]);
      }
      throw FlutterError.fromParts(error);
    }());
    return true;
  }

  void _removeChildData(RestorationBucket child) {
    assert(child._parent == this);
    if (_claimedChildren.remove(child.restorationId) == child) {
      _rawChildren.remove(child.restorationId);
      final List<RestorationBucket>? pendingChildren = _childrenToAdd[child.restorationId];
      if (pendingChildren != null) {
        final RestorationBucket toAdd = pendingChildren.removeLast();
        _finalizeAddChildData(toAdd);
        if (pendingChildren.isEmpty) {
          _childrenToAdd.remove(child.restorationId);
        }
      }
      if (_rawChildren.isEmpty) {
        _rawData.remove(_childrenMapKey);
      }
      _markNeedsSerialization();
      return;
    }
    _childrenToAdd[child.restorationId]?.remove(child);
    if (_childrenToAdd[child.restorationId]?.isEmpty ?? false) {
      _childrenToAdd.remove(child.restorationId);
    }
  }

  void _addChildData(RestorationBucket child) {
    assert(child._parent == this);
    if (_claimedChildren.containsKey(child.restorationId)) {
      // Delay addition until the end of the frame in the hopes that the current
      // owner of the child with the same ID will have given up that child by
      // then.
      _childrenToAdd.putIfAbsent(child.restorationId, () => <RestorationBucket>[]).add(child);
      _markNeedsSerialization();
      return;
    }
    _finalizeAddChildData(child);
    _markNeedsSerialization();
  }

  void _finalizeAddChildData(RestorationBucket child) {
    assert(_claimedChildren[child.restorationId] == null);
    assert(_rawChildren[child.restorationId] == null);
    _claimedChildren[child.restorationId] = child;
    _rawChildren[child.restorationId] = child._rawData;
  }

  void _visitChildren(_BucketVisitor visitor, {bool concurrentModification = false}) {
    Iterable<RestorationBucket> children = _claimedChildren.values.followedBy(
      _childrenToAdd.values.expand((List<RestorationBucket> buckets) => buckets),
    );
    if (concurrentModification) {
      children = children.toList(growable: false);
    }
    children.forEach(visitor);
  }

  // Bucket management

  /// Changes the restoration ID under which the bucket is (or will be) stored
  /// in its parent to `newRestorationId`.
  ///
  /// No-op if the bucket is already stored under the provided id.
  ///
  /// If another owner has already claimed a bucket with the provided `newId` an
  /// exception will be thrown at the end of the current frame unless the other
  /// owner has deleted its bucket by calling [dispose], [rename]ed it using
  /// another ID, or has moved it to a new parent via [adoptChild].
  void rename(String newRestorationId) {
    assert(_debugAssertNotDisposed());
    if (newRestorationId == restorationId) {
      return;
    }
    _parent?._removeChildData(this);
    _restorationId = newRestorationId;
    _parent?._addChildData(this);
  }

  // TODO(polina-c): stop duplicating code across disposables
  // https://github.com/flutter/flutter/issues/137435
  /// Dispatches event of object creation to [FlutterMemoryAllocations.instance].
  void _maybeDispatchObjectCreation() {
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/services.dart',
        className: '$RestorationBucket',
        object: this,
      );
    }
  }

  /// Deletes the bucket and all the data stored in it from the bucket
  /// hierarchy.
  ///
  /// After [dispose] has been called, the data stored in this bucket and its
  /// children are no longer part of the app's restoration data. The data
  /// originally stored in the bucket will not be available again when the
  /// application is restored to this state in the future. It is up to the
  /// owners of the children to either move them (via [adoptChild]) to a new
  /// parent that is still part of the bucket hierarchy or to [dispose] of them
  /// as well.
  ///
  /// This method must only be called by the object's owner.
  void dispose() {
    assert(_debugAssertNotDisposed());
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _visitChildren(_dropChild, concurrentModification: true);
    _claimedChildren.clear();
    _childrenToAdd.clear();
    _parent?._removeChildData(this);
    _parent = null;
    _updateManager(null);
    _debugDisposed = true;
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, 'RestorationBucket')}(restorationId: $restorationId, owner: $debugOwner)';

  bool _debugDisposed = false;
  bool _debugAssertNotDisposed() {
    assert(() {
      if (_debugDisposed) {
        throw FlutterError(
          'A $runtimeType was used after being disposed.\n'
          'Once you have called dispose() on a $runtimeType, it can no longer be used.',
        );
      }
      return true;
    }());
    return true;
  }
}

/// Returns true when the provided `object` is serializable for state
/// restoration.
///
/// Should only be called from within asserts. Always returns false outside
/// of debug builds.
bool debugIsSerializableForRestoration(Object? object) {
  bool result = false;

  assert(() {
    try {
      const StandardMessageCodec().encodeMessage(object);
      result = true;
    } catch (error) {
      // This is only used in asserts, so reporting the exception isn't
      // particularly useful, since the assert itself will likely fail.
      result = false;
    }
    return true;
  }());

  return result;
}
