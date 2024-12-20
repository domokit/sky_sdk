// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of dart.ui;

/// Static methods to allow for simple sharing of [SendPort]s across [Isolate]s.
///
/// All isolates share a global mapping of names to ports. An isolate can
/// register a [SendPort] with a given name using [registerPortWithName];
/// another isolate can then look up that port using [lookupPortByName].
///
/// To create a [SendPort], first create a [ReceivePort], then use
/// [ReceivePort.sendPort].
///
/// Since multiple isolates can each obtain the same [SendPort] associated with
/// a particular [ReceivePort], the protocol built on top of this mechanism
/// should typically consist of a single message. If more elaborate two-way
/// communication or multiple-message communication is necessary, it is
/// recommended to establish a separate communication channel in that first
/// message (e.g. by passing a dedicated [SendPort]).
abstract final class IsolateNameServer {
  /// Looks up the [SendPort] associated with a given name.
  ///
  /// Returns null if the name does not exist. To register the name in the first
  /// place, consider [registerPortWithName].
  ///
  /// The `name` argument must not be null.
  static SendPort? lookupPortByName(String name) {
    return _lookupPortByName(name);
  }

  /// Registers a [SendPort] with a given name.
  ///
  /// Returns true if registration is successful, and false if the name entry
  /// already existed (in which case the earlier registration is left
  /// unchanged). To remove a registration, consider [removePortNameMapping].
  ///
  /// Once a port has been registered with a name, it can be obtained from any
  /// [Isolate] using [lookupPortByName].
  ///
  /// Multiple isolates should avoid attempting to register ports with the same
  /// name, as there is an inherent race condition in doing so.
  ///
  /// The `port` and `name` arguments must not be null.
  static bool registerPortWithName(SendPort port, String name) {
    return _registerPortWithName(port, name);
  }

  /// Removes a name-to-[SendPort] mapping given its name.
  ///
  /// Returns true if the mapping was successfully removed, false if the mapping
  /// did not exist. To add a registration, consider [registerPortWithName].
  ///
  /// Generally, removing a port name mapping is an inherently racy operation
  /// (another isolate could have obtained the name just prior to the name being
  /// removed, and thus would still be able to communicate over the port even
  /// after it has been removed).
  ///
  /// The `name` argument must not be null.
  static bool removePortNameMapping(String name) {
    return _removePortNameMapping(name);
  }

  @Native<Handle Function(Handle)>(symbol: 'IsolateNameServerNatives::LookupPortByName')
  external static SendPort? _lookupPortByName(String name);

  @Native<Bool Function(Handle, Handle)>(symbol: 'IsolateNameServerNatives::RegisterPortWithName')
  external static bool _registerPortWithName(SendPort port, String name);

  @Native<Bool Function(Handle)>(symbol: 'IsolateNameServerNatives::RemovePortNameMapping')
  external static bool _removePortNameMapping(String name);
}
