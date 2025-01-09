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

typedef EventListener = JSFunction;
typedef XMLHttpRequestResponseType = String;

@JS()
external Document get document;

@JS()
external Window get window;

extension type CSSStyleDeclaration._(JSObject _) implements JSObject {
  external set backgroundColor(String value);
  external String get backgroundColor;
  external set border(String value);
  external String get border;
  external set height(String value);
  external String get height;
  external set width(String value);
  external String get width;
}

extension type CSSStyleSheet._(JSObject _) implements JSObject {
  external int insertRule(String rule, [int index]);
}

extension type Document._(JSObject _) implements JSObject {
  external Element createElement(String localName, [JSAny options]);
  external Range createRange();
  external HTMLHeadElement? get head;
}

extension type DOMTokenList._(JSObject _) implements JSObject {
  external void add(String tokens);
}

extension type Element._(JSObject _) implements Node, JSObject {
  external DOMTokenList get classList;
  external void append(JSAny nodes);
  external void remove();
}

extension type Event._(JSObject _) implements JSObject {}

extension type EventTarget._(JSObject _) implements JSObject {
  external void addEventListener(String type, EventListener? callback, [JSAny options]);
}

extension type HTMLElement._(JSObject _) implements Element, JSObject {
  external String get innerText;
  external set innerText(String value);
  external CSSStyleDeclaration get style;
  external HTMLElement cloneNode(bool deep);
}

extension type HTMLHeadElement._(JSObject _) implements HTMLElement, JSObject {}

extension type HTMLStyleElement._(JSObject _) implements HTMLElement, JSObject {
  external CSSStyleSheet? get sheet;
}

extension type HTMLImageElement._(JSObject _) implements HTMLElement, JSObject {
  external String get src;
  external set src(String value);
  external num get naturalWidth;
  external num get naturalHeight;
  external JSPromise<JSAny?> decode();
}

extension type HTMLCanvasElement._(JSObject _) implements HTMLElement, JSObject {
  external int get width;
  external set width(int value);
  external int get height;
  external set height(int value);
  external String toDataURL();
}

extension type MediaQueryList._(JSObject _) implements EventTarget, JSObject {
  external bool get matches;
}

extension type MouseEvent._(JSObject _) implements JSObject {
  external num get offsetX;
  external num get offsetY;
  external int get button;
}

extension type Navigator._(JSObject _) implements JSObject {
  external String get platform;
}

extension type Node._(JSObject _) implements EventTarget, JSObject {}

extension type Range._(JSObject _) implements JSObject {
  external void selectNode(Node node);
}

extension type Selection._(JSObject _) implements JSObject {
  external void addRange(Range range);
  external void removeAllRanges();
}

extension type Window._(JSObject _) implements EventTarget, JSObject {
  external Navigator get navigator;
  external MediaQueryList matchMedia(String query);
  external Selection? getSelection();
  external String get origin;
}

extension type XMLHttpRequest._(JSObject _) implements XMLHttpRequestEventTarget, JSObject {
  external factory XMLHttpRequest();
  external void open(String method, String url, [bool async, String? username, String? password]);
  external void setRequestHeader(String name, String value);
  external void send([JSAny? body]);
  external int get status;
  external set responseType(XMLHttpRequestResponseType value);
  external XMLHttpRequestResponseType get responseType;
  external JSAny? get response;
}

extension type XMLHttpRequestEventTarget._(JSObject _) implements EventTarget, JSObject {}
