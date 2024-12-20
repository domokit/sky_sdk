// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import 'base/config.dart';
import 'base/context.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/platform.dart';
import 'version.dart';

/// A class that represents global (non-project-specific) internal state that
/// must persist across tool invocations.
abstract class PersistentToolState {
  factory PersistentToolState({
    required FileSystem fileSystem,
    required Logger logger,
    required Platform platform,
  }) = _DefaultPersistentToolState;

  factory PersistentToolState.test({required Directory directory, required Logger logger}) =
      _DefaultPersistentToolState.test;

  static PersistentToolState? get instance => context.get<PersistentToolState>();

  /// Whether the welcome message should be redisplayed.
  ///
  /// May give null if the value has not been set.
  bool? get shouldRedisplayWelcomeMessage;
  void setShouldRedisplayWelcomeMessage(bool value); // Enforced nonnull setter.

  /// Returns the last active version for a given [channel].
  ///
  /// If there was no active prior version, returns `null` instead.
  String? lastActiveVersion(Channel channel);

  /// Update the last active version for a given [channel].
  void updateLastActiveVersion(String fullGitHash, Channel channel);

  /// Return the hash of the last active license terms.
  String? get lastActiveLicenseTermsHash;
  void setLastActiveLicenseTermsHash(String value); // Enforced nonnull setter.

  /// Whether this client was already determined to be or not be a bot.
  bool? get isRunningOnBot;
  void setIsRunningOnBot(bool value); // Enforced nonnull setter.
}

class _DefaultPersistentToolState implements PersistentToolState {
  _DefaultPersistentToolState({
    required FileSystem fileSystem,
    required Logger logger,
    required Platform platform,
  }) : _config = Config(_kFileName, fileSystem: fileSystem, logger: logger, platform: platform);

  @visibleForTesting
  _DefaultPersistentToolState.test({required Directory directory, required Logger logger})
    : _config = Config.test(name: _kFileName, directory: directory, logger: logger);

  static const String _kFileName = 'tool_state';
  static const String _kRedisplayWelcomeMessage = 'redisplay-welcome-message';
  static const Map<Channel, String> _lastActiveVersionKeys = <Channel, String>{
    Channel.master: 'last-active-master-version',
    Channel.main: 'last-active-main-version',
    Channel.beta: 'last-active-beta-version',
    Channel.stable: 'last-active-stable-version',
  };
  static const String _kBotKey = 'is-bot';
  static const String _kLicenseHash = 'license-hash';

  final Config _config;

  @override
  bool? get shouldRedisplayWelcomeMessage {
    return _config.getValue(_kRedisplayWelcomeMessage) as bool?;
  }

  @override
  void setShouldRedisplayWelcomeMessage(bool value) {
    _config.setValue(_kRedisplayWelcomeMessage, value);
  }

  @override
  String? lastActiveVersion(Channel channel) {
    final String? versionKey = _versionKeyFor(channel);
    assert(versionKey != null);
    return _config.getValue(versionKey!) as String?;
  }

  @override
  void updateLastActiveVersion(String fullGitHash, Channel channel) {
    final String? versionKey = _versionKeyFor(channel);
    assert(versionKey != null);
    _config.setValue(versionKey!, fullGitHash);
  }

  @override
  String? get lastActiveLicenseTermsHash => _config.getValue(_kLicenseHash) as String?;

  @override
  void setLastActiveLicenseTermsHash(String value) {
    _config.setValue(_kLicenseHash, value);
  }

  String? _versionKeyFor(Channel channel) {
    return _lastActiveVersionKeys[channel];
  }

  @override
  bool? get isRunningOnBot => _config.getValue(_kBotKey) as bool?;

  @override
  void setIsRunningOnBot(bool value) {
    _config.setValue(_kBotKey, value);
  }
}
