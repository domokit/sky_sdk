// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';

import '../persistent_tool_state.dart';
import 'io.dart';
import 'net.dart';
import 'platform.dart';

class BotDetector {
  BotDetector({
    required HttpClientFactory httpClientFactory,
    required Platform platform,
    required PersistentToolState persistentToolState,
  }) : _platform = platform,
       _azureDetector = AzureDetector(httpClientFactory: httpClientFactory),
       _persistentToolState = persistentToolState;

  final Platform _platform;
  final AzureDetector _azureDetector;
  final PersistentToolState _persistentToolState;

  Future<bool> get isRunningOnBot async {
    if ( // Explicitly stated to not be a bot.
    _platform.environment['BOT'] == 'false'
        // Set by the IDEs to the IDE name, so a strong signal that this is not a bot.
        ||
        _platform.environment.containsKey('FLUTTER_HOST')
        // When set, GA logs to a local file (normally for tests) so we don't need to filter.
        ||
        _platform.environment.containsKey('FLUTTER_ANALYTICS_LOG_FILE')) {
      _persistentToolState.setIsRunningOnBot(false);
      return false;
    }

    if (_persistentToolState.isRunningOnBot != null) {
      return _persistentToolState.isRunningOnBot!;
    }

    final bool result =
        _platform.environment['BOT'] == 'true'
        // https://docs.travis-ci.com/user/environment-variables/#Default-Environment-Variables
        ||
        _platform.environment['TRAVIS'] == 'true' ||
        _platform.environment['CONTINUOUS_INTEGRATION'] == 'true' ||
        _platform.environment.containsKey('CI') // Travis and AppVeyor
        // https://www.appveyor.com/docs/environment-variables/
        ||
        _platform.environment.containsKey('APPVEYOR')
        // https://cirrus-ci.org/guide/writing-tasks/#environment-variables
        ||
        _platform.environment.containsKey('CIRRUS_CI')
        // https://docs.aws.amazon.com/codebuild/latest/userguide/build-env-ref-env-vars.html
        ||
        (_platform.environment.containsKey('AWS_REGION') &&
            _platform.environment.containsKey('CODEBUILD_INITIATOR'))
        // https://wiki.jenkins.io/display/JENKINS/Building+a+software+project#Buildingasoftwareproject-belowJenkinsSetEnvironmentVariables
        ||
        _platform.environment.containsKey('JENKINS_URL')
        // https://help.github.com/en/actions/configuring-and-managing-workflows/using-environment-variables#default-environment-variables
        ||
        _platform.environment.containsKey('GITHUB_ACTIONS')
        // Properties on Flutter's Chrome Infra bots.
        ||
        _platform.environment['CHROME_HEADLESS'] == '1' ||
        _platform.environment.containsKey('BUILDBOT_BUILDERNAME') ||
        _platform.environment.containsKey('SWARMING_TASK_ID')
        // Property when running on borg.
        ||
        _platform.environment.containsKey('BORG_ALLOC_DIR')
        // Property when running on Azure.
        ||
        await _azureDetector.isRunningOnAzure;

    _persistentToolState.setIsRunningOnBot(result);
    return result;
  }
}

// Are we running on Azure?
// https://docs.microsoft.com/en-us/azure/virtual-machines/linux/instance-metadata-service
@visibleForTesting
class AzureDetector {
  AzureDetector({required HttpClientFactory httpClientFactory})
    : _httpClientFactory = httpClientFactory;

  static const String _serviceUrl = 'http://169.254.169.254/metadata/instance';

  final HttpClientFactory _httpClientFactory;

  bool? _isRunningOnAzure;

  Future<bool> get isRunningOnAzure async {
    if (_isRunningOnAzure != null) {
      return _isRunningOnAzure!;
    }
    const Duration connectionTimeout = Duration(milliseconds: 250);
    const Duration requestTimeout = Duration(seconds: 1);
    final HttpClient client = _httpClientFactory()..connectionTimeout = connectionTimeout;
    try {
      final HttpClientRequest request = await client
          .getUrl(Uri.parse(_serviceUrl))
          .timeout(requestTimeout);
      request.headers.add('Metadata', true);
      await request.close();
    } on SocketException {
      // If there is an error on the socket, it probably means that we are not
      // running on Azure.
      return _isRunningOnAzure = false;
    } on HttpException {
      // If the connection gets set up, but encounters an error condition, it
      // still means we're on Azure.
      return _isRunningOnAzure = true;
    } on TimeoutException {
      // The HttpClient connected to a host, but it did not respond in a timely
      // fashion. Assume we are not on a bot.
      return _isRunningOnAzure = false;
    } on OSError {
      // The HttpClient might be running in a WSL1 environment.
      return _isRunningOnAzure = false;
    }
    // We got a response. We're running on Azure.
    return _isRunningOnAzure = true;
  }
}
