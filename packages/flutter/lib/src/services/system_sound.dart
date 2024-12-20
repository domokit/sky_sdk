// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'system_channels.dart';

/// A sound provided by the system.
///
/// These sounds may be played with [SystemSound.play].
enum SystemSoundType {
  /// A short indication that a button was pressed.
  click,

  /// A short system alert sound indicating the need for user attention.
  ///
  /// Desktop platforms are the only platforms that support a system alert
  /// sound, so on mobile platforms (Android, iOS), this will be ignored. The
  /// web platform does not support playing any sounds, so this will be
  /// ignored on the web as well.
  alert,

  // If you add new values here, you also need to update the `SoundType` Java
  // enum in `PlatformChannel.java`.
}

/// Provides access to the library of short system specific sounds for common
/// tasks.
abstract final class SystemSound {
  /// Play the specified system sound. If that sound is not present on the
  /// system, the call is ignored.
  ///
  /// The web platform currently does not support playing sounds, so this call
  /// will yield no behavior on that platform.
  static Future<void> play(SystemSoundType type) async {
    await SystemChannels.platform.invokeMethod<void>('SystemSound.play', type.toString());
  }
}
