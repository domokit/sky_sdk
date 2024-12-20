// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'binding.dart';
/// @docImport 'debug.dart';
/// @docImport 'platform.dart';
library;

/// Service extension constants for the foundation library.
///
/// These constants will be used when registering service extensions in the
/// framework, and they will also be used by tools and services that call these
/// service extensions.
///
/// The String value for each of these extension names should be accessed by
/// calling the `.name` property on the enum value.
enum FoundationServiceExtensions {
  /// Name of service extension that, when called, will cause the entire
  /// application to redraw.
  ///
  /// See also:
  ///
  /// * [BindingBase.initServiceExtensions], where the service extension is
  ///   registered.
  reassemble,

  /// Name of service extension that, when called, will terminate the Flutter
  /// application.
  ///
  /// See also:
  ///
  /// * [BindingBase.initServiceExtensions], where the service extension is
  ///   registered.
  exit,

  /// Name of service extension that, when called, will get or set the value of
  /// [connectedVmServiceUri].
  ///
  /// See also:
  ///
  /// * [connectedVmServiceUri], which stores the uri for the connected vm service
  ///   protocol.
  /// * [BindingBase.initServiceExtensions], where the service extension is
  ///   registered.
  connectedVmServiceUri,

  /// Name of service extension that, when called, will get or set the value of
  /// [activeDevToolsServerAddress].
  ///
  /// See also:
  ///
  /// * [activeDevToolsServerAddress], which stores the address for the active
  ///   DevTools server used for debugging this application.
  /// * [BindingBase.initServiceExtensions], where the service extension is
  ///   registered.
  activeDevToolsServerAddress,

  /// Name of service extension that, when called, will change the value of
  /// [defaultTargetPlatform], which controls which [TargetPlatform] that the
  /// framework will execute for.
  ///
  /// See also:
  ///
  /// * [debugDefaultTargetPlatformOverride], which is the flag that this service
  ///   extension exposes.
  /// * [BindingBase.initServiceExtensions], where the service extension is
  ///   registered.
  platformOverride,

  /// Name of service extension that, when called, will override the platform
  /// [Brightness].
  ///
  /// See also:
  ///
  /// * [debugBrightnessOverride], which is the flag that this service
  ///   extension exposes.
  /// * [BindingBase.initServiceExtensions], where the service extension is
  ///   registered.
  brightnessOverride,
}
