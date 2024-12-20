// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This code is copied from `package:web` which still needs its own
// documentation for public members. Since this is a shim that users should not
// use, we ignore this lint for this file.
// ignore_for_file: public_member_api_docs

/// A stripped down version of `package:web` to avoid pinning that repo in
/// Flutter as a dependency.
///
/// These are manually copied over from `package:web` as needed, and should stay
/// in sync with the latest package version as much as possible.
///
/// If missing members are needed, copy them over into the corresponding
/// extension or interface. If missing interfaces/types are needed, copy them
/// over while excluding unnecessary inheritance to make the copy minimal. These
/// types are erased at runtime, so excluding supertypes is safe. If a member is
/// needed that belongs to a supertype, then add the necessary `implements`
/// clause to the subtype when you add that supertype. Keep extensions next to
/// the interface they extend.
library;

import 'dart:js_interop';

@JS()
external Window get window;

extension type Window._(JSObject _) implements JSObject {
  external JSPromise<Response> fetch(JSAny input, [RequestInit init]);
  external Location get location;
  external Window? get parent;
  external void postMessage(JSAny message, JSString targetOrigin, JSArray<JSAny> transfers);

  external JSString? get testSelector;
}

extension type Response._(JSObject _) implements JSObject {
  external JSPromise<JSString> text();
}

extension type RequestInit._(JSObject _) implements JSObject {
  external factory RequestInit({String method, JSAny? body});
}

extension type Location._(JSObject _) implements JSObject {
  external JSString get origin;
}

extension type MessageChannel._(JSObject _) implements JSObject {
  external factory MessageChannel();

  external MessagePort port1;
  external MessagePort port2;
}

extension type MessagePort._(JSObject _) implements JSObject {
  external void addEventListener(JSString eventName, JSFunction callback);
  external void removeEventListener(JSString eventName, JSFunction callback);
  external void postMessage(JSAny? message);

  external void start();
}

extension type Event._(JSObject _) implements JSObject {
  external JSObject? get data;
}
